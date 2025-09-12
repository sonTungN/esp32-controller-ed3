#include <Arduino.h>
#include <PID_v1.h>

#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"

// Motor Forward/Backward Left/Right Pins
#define MOTOR_FL_A 1
#define MOTOR_FL_B 2
#define MOTOR_FR_A 3
#define MOTOR_FR_B 4
#define MOTOR_BL_A 5
#define MOTOR_BL_B 6
#define MOTOR_BR_A 7
#define MOTOR_BR_B 8

#define MOTOR_FL_CHANNEL 0
#define MOTOR_FR_CHANNEL 1
#define MOTOR_BL_CHANNEL 2
#define MOTOR_BR_CHANNEL 3

// Motor PWM Frequency
#define MOTOR_PWM_FREQUENCY 10000
#define MOTOR_PWM_RESOLUTION 8

// Encoder Pins
#define ENC_FL_A 9
#define ENC_FL_B 10
#define ENC_FR_A 11
#define ENC_FR_B 12
#define ENC_BL_A 13
#define ENC_BL_B 14
#define ENC_BR_A 15
#define ENC_BR_B 16

// PID Input, Output, Ref
double kp = 0.1, ki = 10, kd = 0;
double speed_fl = 0, speed_fr = 0, speed_bl = 0, speed_br = 0;
double speed_fl_ref = 0, speed_fr_ref = 0, speed_bl_ref = 0, speed_br_ref = 0;

PID MOTOR_FL_PID(&speed_fl, &motor_cmd_fl, &speed_fl_ref, kp, ki, kd, DIRECT);
PID MOTOR_FR_PID(&speed_fr, &motor_cmd_fr, &speed_fr_ref, kp, ki, kd, DIRECT);
PID MOTOR_BL_PID(&speed_bl, &motor_cmd_bl, &speed_bl_ref, kp, ki, kd, DIRECT);
PID MOTOR_BR_PID(&speed_br, &motor_cmd_br, &speed_br_ref, kp, ki, kd, DIRECT);

// Global Variables for the position of the encoders
volatile long pos_fl = 0, pos_fr = 0, pos_bl = 0, pos_br = 0;

// Variables for the motor commands
double motor_cmd_fl = 0, motor_cmd_fr = 0, motor_cmd_bl = 0, motor_cmd_br = 0;

void init_setup_motor() {
  // Configure the motor pins as output
  pinMode(MOTOR_FL_A, OUTPUT);
  pinMode(MOTOR_FL_B, OUTPUT);
  pinMode(MOTOR_FR_A, OUTPUT);
  pinMode(MOTOR_FR_B, OUTPUT);
  pinMode(MOTOR_BL_A, OUTPUT);
  pinMode(MOTOR_BL_B, OUTPUT);
  pinMode(MOTOR_BR_A, OUTPUT);
  pinMode(MOTOR_BR_B, OUTPUT);

  // Set up the PWM channels for the motors
  ledcSetup(MOTOR_FL_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_FR_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_BL_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_BR_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
}

void send_pwm(double motor_cmd, int pin_A, int pin_B, int channel) {
  if (motor_cmd < 0) {
    ledcAttachPin(pin_B, channel);
    ledcDetachPin(pin_A);
    digitalWrite(pin_A, LOW);
    ledcWrite(channel, abs((int) motor_cmd));

  } else {
    ledcAttachPin(pin_A, channel);
    ledcDetachPin(pin_B);
    digitalWrite(pin_B, LOW);
    ledcWrite(channel, abs((int) motor_cmd));
  }
}

void run_motors() {
  send_pwm(motor_cmd_fl, MOTOR_FL_A, MOTOR_FL_B, MOTOR_FL_CHANNEL);
  send_pwm(motor_cmd_fr, MOTOR_FR_A, MOTOR_FR_B, MOTOR_FR_CHANNEL);
  send_pwm(motor_cmd_bl, MOTOR_BL_A, MOTOR_BL_B, MOTOR_BL_CHANNEL);
  send_pwm(motor_cmd_br, MOTOR_BR_A, MOTOR_BR_B, MOTOR_BR_CHANNEL);
}

void init_setup_encoders()
{
  pinMode(ENC_FL_A, INPUT);
  pinMode(ENC_FL_B, INPUT);
  pinMode(ENC_FR_A, INPUT);
  pinMode(ENC_FR_B, INPUT);
  pinMode(ENC_BL_A, INPUT);
  pinMode(ENC_BL_B, INPUT);
  pinMode(ENC_BR_A, INPUT);
  pinMode(ENC_BR_B, INPUT);

  // Attach Interrupt for the encoders
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), read_FL_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), read_FR_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), read_BL_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), read_BR_encoder, RISING);
}

void read_FL_encoder() {
  int b = digitalRead(ENC_FL_B);
  pos_fl = (b > 0) ? ++pos_fl : --pos_fl;
}

void read_FR_encoder() {
  int b = digitalRead(ENC_FR_B);
  pos_fr = (b > 0) ? ++pos_fr : --pos_fr;
}

void read_BL_encoder() {
  int b = digitalRead(ENC_BL_B);
  pos_bl = (b > 0) ? ++pos_bl : --pos_bl;
}

void read_BR_encoder() {
  int b = digitalRead(ENC_BR_B);
  pos_br = (b > 0) ? ++pos_br : --pos_br;
}

void init_pid() {
  MOTOR_FL_PID.SetMode(AUTOMATIC);
  MOTOR_FL_PID.SetOutputLimits(-255, 255);
  MOTOR_FL_PID.SetSampleTime(10);

  MOTOR_FR_PID.SetMode(AUTOMATIC);
  MOTOR_FR_PID.SetOutputLimits(-255, 255);
  MOTOR_FR_PID.SetSampleTime(10);

  MOTOR_BL_PID.SetMode(AUTOMATIC);
  MOTOR_BL_PID.SetOutputLimits(-255, 255);
  MOTOR_BL_PID.SetSampleTime(10);

  MOTOR_BR_PID.SetMode(AUTOMATIC);
  MOTOR_BR_PID.SetOutputLimits(-255, 255);
  MOTOR_BR_PID.SetSampleTime(10);
}

void compute_pid() {
  MOTOR_FL_PID.SetTunings(kp, ki, kd);
  MOTOR_FR_PID.SetTunings(kp, ki, kd);
  MOTOR_BL_PID.SetTunings(kp, ki, kd);
  MOTOR_BR_PID.SetTunings(kp, ki, kd);

  MOTOR_FL_PID.Compute();
  MOTOR_FR_PID.Compute();
  MOTOR_BL_PID.Compute();
  MOTOR_BR_PID.Compute();
}