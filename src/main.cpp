#include <Adafruit_MLX90614.h>
#include <Arduino.h>
#include <Motor.h>
#include <VL53L0X.h>
#include <Wire.h>

#define R 0
#define L 1

VL53L0X vl53l0x;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(0x5A);
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(0x55);  // I2Cアドレス書き換え済み

/*
#define DRV8835_R_DIR 0
#define DRV8835_R_PWM 1
#define DRV8835_L_DIR 2
#define DRV8835_L_PWM 3
4 : SDA
5 : SCL
#define HALL_R 6
#define HALL_L 7
*/
const unsigned char HALL_PINS[] = {6, 7};
const float MOTOR_SPEED = 250;

unsigned long t = 0;
unsigned long reachedTimeZeroDeg[] = {0, 0};
unsigned long motorSlowedTime[] = {0, 0};

int getOtherSide(unsigned char side) { return side == R ? L : R; }

void led(bool on) {
  int v = on ? LOW : HIGH;  // HIGHがOFF
  digitalWrite(LED_BUILTIN, v);
}

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  led(false);

  Motor::init();

  t = millis();
}

bool isHallActive(unsigned char side) {
  bool active = analogRead(HALL_PINS[side]) > 255;
  if (side == R && active) {
    led(true);
  } else {
    led(false);
  }
  return active;
}

void adjustSpeed(int side, int motorSpeed, bool isBack = false) {
  unsigned long lastTime = reachedTimeZeroDeg[side];
  unsigned long time = millis();
  reachedTimeZeroDeg[side] = time;

  if (lastTime) {
    // 1回転にかかった時間
    unsigned long rotateTime = time - lastTime;

    // 逆側がどれくらい進んでいるか
    int otherSide = getOtherSide(side);
    unsigned long lastTimeOther = reachedTimeZeroDeg[otherSide];
    unsigned long currentTimeOther = time - lastTimeOther;

    /*
    Serial.print("rotatetime");
    Serial.print(side);
    Serial.print(":");
    Serial.print(rotateTime);
    Serial.print("/");
    Serial.print(currentTimeOther);
    Serial.println("");
    */

    // 逆側が半分も進んでいない場合こっちを遅くする
    unsigned int timeThreshold = 50;  // 位相差πに近い場合はslowしない
    if (!isBack) {
      if (rotateTime / 2 + timeThreshold > currentTimeOther) {
        Motor::start(side, motorSpeed * 0.8);
        motorSlowedTime[side] = millis();
      } else if (rotateTime / 2 < currentTimeOther + timeThreshold) {
        Motor::start(otherSide, motorSpeed * 0.8);
        motorSlowedTime[otherSide] = millis();
      }
    } else {
      // 後進中は逆の判定
      if (rotateTime / 2 < currentTimeOther + timeThreshold) {
        Motor::start(side, motorSpeed * 0.8);
        motorSlowedTime[side] = millis();
      } else if (rotateTime / 2 + timeThreshold > currentTimeOther) {
        Motor::start(otherSide, motorSpeed * 0.8);
        motorSlowedTime[otherSide] = millis();
      }
    }

    /*
    if(side == R){
      Serial.print(time);
      Serial.print("\t");
      Serial.println(time - lastTime);
    }
    */
  }
}
void walk(float motorSpeeds[2], unsigned int steps, bool isBack = false) {
  unsigned int reachedCountZeroDeg[] = {0, 0};
  bool hallWasActive[] = {false, false};

  // 計測時間をリセット
  reachedTimeZeroDeg[R] = 0;
  reachedTimeZeroDeg[L] = 0;

  Motor::start(R, motorSpeeds[R]);
  Motor::start(L, motorSpeeds[L]);

  while (true) {
    int sides[] = {R, L};
    for (int side : sides) {
      // slowしてから一定時間たっていたら元のスピードに戻す
      if (millis() - motorSlowedTime[side] > 300) {
        // Serial.print(side);
        // Serial.println(" normal");
        Motor::start(side, motorSpeeds[side]);
      }

      // ホールセンサに反応したら速度調整へ
      bool hallActive = isHallActive(side);
      if (!hallWasActive[side] && hallActive) {
        reachedCountZeroDeg[side] += 1;
        adjustSpeed(side, motorSpeeds[side], isBack);
      }
      hallWasActive[side] = hallActive;
    }

    // steps歩歩いたらbreak
    if (reachedCountZeroDeg[R] > steps && reachedCountZeroDeg[L] > steps) {
      reachedCountZeroDeg[R] = 0;
      reachedCountZeroDeg[L] = 0;
      break;
    }
  }
}

void foward(unsigned int steps) {
  float motorSpeeds[] = {MOTOR_SPEED, MOTOR_SPEED};
  walk(motorSpeeds, steps, false);
}

void backward(unsigned int steps) {
  float motorSpeeds[] = {-1 * MOTOR_SPEED, -1 * MOTOR_SPEED};
  walk(motorSpeeds, steps, false);
}

void turn(unsigned int side, unsigned int steps) {
  float speedR = side == R ? MOTOR_SPEED : -1 * MOTOR_SPEED;
  float speedL = -1 * speedR;
  float motorSpeeds[] = {speedR, speedL};
  walk(motorSpeeds, steps, true);
}

void stop() {
  int sides[] = {R, L};
  for (int side : sides) {
    Motor::stop(side);
  }
}

void speedTest(unsigned char side, int speed, unsigned int steps) {
  Serial.println("");
  Serial.print(side == 0 ? "R" : "L");
  Serial.println(speed);

  unsigned long reachedTimeZeroDegTest = 0;
  unsigned int reachedCountZeroDeg = 0;
  bool hallWasActive = false;

  Motor::start(side, speed);

  while (true) {
    // ホールセンサに反応したら速度調整へ
    bool hallActive = isHallActive(side);
    if (!hallWasActive && hallActive) {
      reachedCountZeroDeg += 1;

      // 1周にかかった時間を出力
      unsigned long lastTime = reachedTimeZeroDegTest;
      unsigned long time = millis();

      Serial.print(side);
      Serial.print(" ");
      Serial.println(time - lastTime);
      reachedTimeZeroDegTest = time;
    }
    hallWasActive = hallActive;

    // steps歩歩いたらbreak
    if (reachedCountZeroDeg > steps) {
      reachedCountZeroDeg = 0;
      break;
    }
  }
  stop();
}

void speedTests() {
  delay(5000);

  speedTest(R, 250, 4);
  speedTest(R, -250, 4);
  speedTest(L, 250, 4);
  speedTest(L, -250, 4);
  speedTest(R, 200, 4);
  speedTest(R, -200, 4);
  speedTest(L, 200, 4);
  speedTest(L, -200, 4);

  speedTest(L, 250, 4);
  speedTest(L, 240, 4);
  speedTest(L, 230, 4);
  speedTest(L, 220, 4);
  speedTest(L, 210, 4);
  speedTest(L, 200, 4);

  speedTest(L, -250, 4);
  speedTest(L, -240, 4);
  speedTest(L, -230, 4);
  speedTest(L, -220, 4);
  speedTest(L, -210, 4);
  speedTest(L, -200, 4);

  speedTest(L, -190, 4);
  speedTest(L, -180, 4);
  speedTest(L, -170, 4);
  speedTest(L, -160, 4);

  speedTest(L, -150, 4);
  speedTest(L, -140, 4);
  speedTest(L, -130, 4);
  speedTest(L, -120, 4);
  speedTest(L, -110, 4);
  speedTest(L, -100, 4);

  speedTest(L, -90, 4);
  speedTest(L, -80, 4);
  speedTest(L, -70, 4);
  speedTest(L, -60, 4);
  speedTest(L, -50, 4);
}

void loop() {
  foward(8);
  stop();
  delay(1000);

  backward(8);
  stop();
  delay(1000);

  turn(R, 8);
  stop();
  delay(1000);

  turn(L, 8);
  stop();
  delay(1000);
}