#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"

float received_angle = 0.0f;
float received_velocity = 0.0f;
float actual_velocity = 0.0f;
float actual_position = 0.0f;
float follow_error = 0.0f;
float motor_enable_offset = 0.0f;

float phaseA = 0.0f;
float phaseb = 0.0F;

unsigned long start;
unsigned long finish;
unsigned long looptime;

#define SENSOR_nCS PB9  // Braun
#define CAL_EN PB3
#define EN_PIN PB14

#define ENCODER_PPR 8000
#define ENCODER_PIN_A PB6
#define ENCODER_PIN_B PB7

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50, 2.2f, 8.0f, 0.0045f);  //2,2Ohm, 8KV, 0,0045H
StepperMotor motor = StepperMotor(50);  //2,2Ohm, 8KV, 0,0045H
StepperDriver4PWM driver = StepperDriver4PWM(0, 1, 2, 3);    //PA0, PA1, PA2, PA3

SPISettings myMT6835SPISettings(4000000, MT6835_BITORDER, SPI_MODE3);   //MOSI → PA7, Grün  MISO → PA6, Weiß  SCLK → PA5, Blau
MagneticSensorMT6835 sensor = MagneticSensorMT6835(SENSOR_nCS, myMT6835SPISettings);
STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, ENCODER_PIN_A, ENCODER_PIN_B);

LowsideCurrentSense current_sense  = LowsideCurrentSense(185.0f, PB1, PB0);    //185mV / A

StepDirListener step_dir = StepDirListener(PB12, PB13, 2*PI/3200);
void onStep() { step_dir.handle(); }

uint8_t calibrateEncoder(void)
{
  motor.voltage_limit = 7;
	motor.controller = MotionControlType::velocity_openloop;

	MT6835Options4 currentSettings = sensor.getOptions4();
	currentSettings.autocal_freq = 0x5;
	sensor.setOptions4(currentSettings);

	uint32_t calTime = micros();
	while ((micros() - calTime) < 120000000)
	{
		motor.move(10);

		if ((micros() -calTime) > 5000000)
		{
			// after motor is spinning at constant speed, enable calibration.
      digitalWrite(LED_BUILTIN, HIGH);
			digitalWrite(CAL_EN, HIGH);
		}
	}
  digitalWrite(LED_BUILTIN, LOW);
	digitalWrite(CAL_EN, LOW);

  motor.controller = MotionControlType::angle;
  motor.voltage_limit = 12;

	return sensor.getCalibrationStatus();
}

//Command settings
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void calibrateEnc(char* cmd) { 
  
  uint8_t calibrationResult = calibrateEncoder();
	if (calibrationResult == 0x3){
		  SIMPLEFOC_DEBUG("Encoder self calibration successful.");
	}
	else{
		SIMPLEFOC_DEBUG("Encoder self calibration failed! Result: %#02x", calibrationResult);
	}
  
}

void setup() {

  pinMode(EN_PIN, INPUT);
  pinMode(CAL_EN, OUTPUT);
  digitalWrite(CAL_EN, LOW);

  // use monitoring with serial 
  Serial.begin(115200);
  delay(3000);

  sensor.init();
  sensor.setABZEnabled(ENABLE);
  sensor.setABZResolution(ENCODER_PPR);
  sensor.writeEEPROM();

  SimpleFOCDebug::enable(&Serial);

  encoder.init();
  motor.linkSensor(&sensor);

  //Driver
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 24.0f;
  driver.init();
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  current_sense.init();
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);

  motor.voltage_sensor_align = 7;

  //FOC model selection
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;
  motor.feed_forward_velocity = received_velocity;

  //Limits
  motor.velocity_limit = 999;
  motor.voltage_limit = 24.0f;
  motor.current_limit = 1.5f;

  //Velocity → deaktiviert in StepperMotor.cpp
  motor.PID_velocity.P = 0.43f;
  motor.PID_velocity.I = 25;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 0;
  motor.LPF_velocity.Tf = 0.01f;
  
  //Angle
  motor.P_angle.P = 150; 
  motor.P_angle.I = 0;  
  motor.P_angle.D = 0;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf = 0;

  //Current
  motor.PID_current_q.P = 2.5f;                      
  motor.PID_current_q.I = 120;
  motor.LPF_current_q.Tf = 1.0f/(300*_2PI);
  motor.PID_current_d.P = 2.5f;
  motor.PID_current_d.I = 120;
  motor.LPF_current_d.Tf = 1.0f/(300*_2PI);

  step_dir.init();
  step_dir.enableInterrupt(onStep);
  step_dir.attach(&received_angle, &received_velocity);  

  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 2.49f;

  motor.init();
  motor.initFOC();

  command.add('M', onMotor, "motor");
  command.add('C', calibrateEnc, "Calibrate Encoder");

  Serial.println(F("Motor ready."));
  motor.disable();

}

void loop() {

  if((digitalRead(EN_PIN) == HIGH) && (motor.enabled == 0)){

    motor_enable_offset = motor.shaft_angle - received_angle;
    motor.enable();
  }
  if((digitalRead(EN_PIN) == LOW) && (motor.enabled == 1)){

    motor.disable();
  }

  start = micros();
  motor.loopFOC();
  actual_position = motor.shaft_angle;
  actual_velocity = motor.shaft_velocity;
  follow_error = received_angle + motor_enable_offset;
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  phaseA = currents.a * 1000;
  phaseb = currents.b * 1000;
  motor.move(received_angle + motor_enable_offset);
  
  command.run();
  finish = micros();
  looptime = finish - start;
  step_dir.update();
}