#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

class Motor {
 public:
  static void init();
  static void start(int side, float speed);
  static void stop(int side);

 private:
  static const unsigned char DRV8835_PINS[][2];
  static const unsigned char DRV8835_DIR;
  static const unsigned char DRV8835_PWM;
  static float motorSpeedRatio;
};
#endif