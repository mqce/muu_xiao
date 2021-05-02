#include "Motor.h"
const unsigned char Motor::DRV8835_DIR = 0;
const unsigned char Motor::DRV8835_PWM = 1;
const unsigned char Motor::DRV8835_PINS[][2] = {{0, 1}, {2, 3}};
float Motor::motorSpeedRatio = 0.85;

void Motor::init() {
  digitalWrite(DRV8835_PINS[0][DRV8835_PWM], LOW);
  pinMode(DRV8835_PINS[0][DRV8835_PWM], OUTPUT);
  digitalWrite(DRV8835_PINS[0][DRV8835_PWM], LOW);

  digitalWrite(DRV8835_PINS[1][DRV8835_PWM], LOW);
  pinMode(DRV8835_PINS[1][DRV8835_PWM], OUTPUT);
  digitalWrite(DRV8835_PINS[1][DRV8835_PWM], LOW);

  digitalWrite(DRV8835_PINS[0][DRV8835_DIR], LOW);
  pinMode(DRV8835_PINS[0][DRV8835_DIR], OUTPUT);
  digitalWrite(DRV8835_PINS[0][DRV8835_DIR], LOW);

  digitalWrite(DRV8835_PINS[1][DRV8835_DIR], LOW);
  pinMode(DRV8835_PINS[1][DRV8835_DIR], OUTPUT);
  digitalWrite(DRV8835_PINS[1][DRV8835_DIR], LOW);
}

void Motor::start(int side, int speed) {
  int pinDir = DRV8835_PINS[side][DRV8835_DIR];
  int pinPwm = DRV8835_PINS[side][DRV8835_PWM];

  // foward or backward
  if (speed > 0) {
    if (side == 0) {
      speed = speed * 0.81;
    }
    speed = 255 - speed;

    analogWrite(pinPwm, speed);
    digitalWrite(pinDir, HIGH);
  } else {
    if (side == 0) {
      speed = speed * 0.5;
    }
    speed = -speed;

    analogWrite(pinPwm, speed);
    digitalWrite(pinDir, LOW);
  }
}

void Motor::stop(int side) {
  int pinDir = DRV8835_PINS[side][DRV8835_DIR];
  int pinPwm = DRV8835_PINS[side][DRV8835_PWM];

  digitalWrite(pinDir, LOW);
  analogWrite(pinPwm, 0);
}
