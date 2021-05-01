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
const unsigned int MOTOR_SPEED_NORMAL = 200;
const unsigned int MOTOR_SPEED_SLOW = 100;

unsigned long t = 0;

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

unsigned long reachedTimeZeroDeg[] = {0, 0};
unsigned long motorSlowedTime[] = {0, 0};

void adjustSpeed(int side) {
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
    if (rotateTime / 2 + timeThreshold > currentTimeOther) {
      Motor::start(side, MOTOR_SPEED_SLOW);
      motorSlowedTime[side] = millis();
    } else if (rotateTime / 2 < currentTimeOther + timeThreshold) {
      Motor::start(otherSide, MOTOR_SPEED_SLOW);
      motorSlowedTime[otherSide] = millis();
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
void walk(unsigned int steps) {
  unsigned int reachedCountZeroDeg[] = {0, 0};

  // 計測時間をリセット
  reachedTimeZeroDeg[R] = 0;
  reachedTimeZeroDeg[L] = 0;

  Motor::start(R, MOTOR_SPEED_NORMAL);
  Motor::start(L, MOTOR_SPEED_NORMAL);

  unsigned long hallWasActive[] = {false, false};

  while (true) {
    int sides[] = {R, L};
    for (int side : sides) {
      // slowしてから一定時間たっていたら元のスピードに戻す
      if (millis() - motorSlowedTime[side] > 200) {
        // Serial.print(side);
        // Serial.println(" normal");
        Motor::start(side, MOTOR_SPEED_NORMAL);
      }

      // ホールセンサに反応したら速度調整へ
      bool hallActive = isHallActive(side);
      if (!hallWasActive[side] && hallActive) {
        reachedCountZeroDeg[side] += 1;
        adjustSpeed(side);
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

void turn() {}

void stop() {
  Motor::stop(R);
  Motor::stop(L);
}

void loop() {
  walk(4);
  stop();
  delay(1000);
}