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

// Global Variables for the position of the encoders
volatile long pos_fl = 0, pos_fr = 0, pos_bl = 0, pos_br = 0;

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

void init_setup_encoders() {
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