#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <VL53L0X.h>

#define R 0
#define L 1
#define DRV8835_DIR 0
#define DRV8835_PWM 1


int getOtherSide(int side){
  return side == R ? L : R;
}
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

int DRV8835_PINS[][2] = { {0, 1}, {2, 3} };
int HALL_PINS[] = {6, 7};

unsigned long t = 0;

const unsigned int MOTOR_SPEED_NORMAL = 200;
const unsigned int MOTOR_SPEED_SLOW = 100;
const float motorSpeedRatio = 0.85;

bool isMotorRunning[] = {false,false};
int motorSpeed[] = {MOTOR_SPEED_NORMAL, MOTOR_SPEED_NORMAL};

VL53L0X vl53l0x;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(0x5A);
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(0x55);



void led(bool on){
  int v = on ? LOW : HIGH;// HIGHがOFF
  digitalWrite(LED_BUILTIN, v);
}

void initMotors(){
  digitalWrite(DRV8835_PINS[R][DRV8835_PWM], LOW);
  pinMode(DRV8835_PINS[R][DRV8835_PWM], OUTPUT);
  digitalWrite(DRV8835_PINS[R][DRV8835_PWM], LOW);

  digitalWrite(DRV8835_PINS[L][DRV8835_PWM], LOW);
  pinMode(DRV8835_PINS[L][DRV8835_PWM], OUTPUT);
  digitalWrite(DRV8835_PINS[L][DRV8835_PWM], LOW);
  
  digitalWrite(DRV8835_PINS[R][DRV8835_DIR], LOW);
  pinMode(DRV8835_PINS[R][DRV8835_DIR], OUTPUT);
  digitalWrite(DRV8835_PINS[R][DRV8835_DIR], LOW);
  
  digitalWrite(DRV8835_PINS[L][DRV8835_DIR], LOW);
  pinMode(DRV8835_PINS[L][DRV8835_DIR], OUTPUT);
  digitalWrite(DRV8835_PINS[L][DRV8835_DIR], LOW);
}

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  led(false);

  initMotors();

  t = millis();
  
/*
  mlx1.begin();
  mlx2.begin();
  
  vl53l0x.setTimeout(1000);
  vl53l0x.init();
  vl53l0x.startContinuous();
*/
}


void test() {
  /*
  int distance = vl53l0x.readRangeContinuousMillimeters();
  Serial.print("mlx1: ");
  Serial.print(mlx1.readAmbientTempC());
  Serial.print("|");
  Serial.print(mlx1.readObjectTempC());
  Serial.println();
  Serial.print("mlx2: ");
  Serial.print(mlx2.readAmbientTempC());
  Serial.print("|");
  Serial.print(mlx2.readObjectTempC());
  Serial.println();
  Serial.print("vlx:");
  Serial.print(distance);
  Serial.println();
  Serial.print("hall:");
  Serial.println(analogRead(HALL_R));
  Serial.println();
  if (vl53l0x.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
  // run M1 motor with positive speed
  
  if(analogRead(HALL_R) > 255){
    digitalWrite(LED_BUILTIN, LOW);
  }else{
    digitalWrite(LED_BUILTIN, HIGH);
  }
  
  if(distance < 50){
    analogWrite(DRV8835_R_DIR, HIGH);
    analogWrite(DRV8835_R_DIR, HIGH);
    delay(2);
    analogWrite(DRV8835_A_PWM, 255);
    analogWrite(DRV8835_B_PWM, 255);
  }
  
  delay(500);
  analogWrite(DRV8835_A_PWM, 0);
  analogWrite(DRV8835_B_PWM, 0);
  delay(500);
  */

}

void startMotor(int side, int speed){
  int pinDir = DRV8835_PINS[side][DRV8835_DIR];
  int pinPwm = DRV8835_PINS[side][DRV8835_PWM];

  if(isMotorRunning[side]){
    // return;
  }
  isMotorRunning[side] = true;
  
  if(side==R){
    speed = speed * motorSpeedRatio;
  }
  // foward or backward
  if(speed > 0){
    speed = 255 - speed;
    analogWrite(pinPwm, speed);
    digitalWrite(pinDir, HIGH);
  }else{
    speed = -speed;
    analogWrite(pinPwm, speed);
    digitalWrite(pinDir, LOW);
  }
}

void stopMotor(int side){
  int pinDir = DRV8835_PINS[side][DRV8835_DIR];
  int pinPwm = DRV8835_PINS[side][DRV8835_PWM];

  if(!isMotorRunning[side]){
    // return;
  }
  isMotorRunning[side] = false;
  digitalWrite(pinDir, LOW);
  analogWrite(pinPwm, 0);
}

bool isHallActive(int side){
  bool active = analogRead(HALL_PINS[side]) > 255;
  if(side == R && active){
    led(true);
  }else{
    led(false);
  }
  return active;
}

unsigned int reachedCountZeroDeg[] = {0,0};
unsigned long reachedTimeZeroDeg[] = {0,0};
unsigned long motorSlowedTime[] = {0, 0};
unsigned long rotateTime[] = {0,0};

void calcSpeed(int side){
  reachedCountZeroDeg[side] +=1;
  unsigned long lastTime = reachedTimeZeroDeg[side];
  unsigned long time = millis();
  reachedTimeZeroDeg[side] = time;

  if(lastTime){
    // 1回転にかかった時間
    unsigned long rotateTime = time - lastTime;
    
    // 逆側がどれくらい進んでいるか
    int otherSide = getOtherSide(side);
    unsigned long lastTimeOther = reachedTimeZeroDeg[otherSide];
    unsigned long currentTimeOther = time - lastTimeOther;

    // 逆側が半分も進んでいない場合こっちを遅くする
    /*
    Serial.print("rotatetime");
    Serial.print(side);
    Serial.print(":");
    Serial.print(rotateTime);
    Serial.print("/");
    Serial.print(currentTimeOther);
    Serial.println("");
    */
    int timeThreshold = 50;// 位相差πに近い場合はslowしない
    if(rotateTime/2 + timeThreshold > currentTimeOther){
      Serial.print(side);
      Serial.println(" slow");
      startMotor(side, MOTOR_SPEED_SLOW);
      motorSlowedTime[side] = millis();
    }else if(rotateTime/2 < currentTimeOther + timeThreshold){
      startMotor(otherSide, MOTOR_SPEED_SLOW);
      motorSlowedTime[otherSide] = millis();
    }

    if(side == R){
      Serial.print(time);
      Serial.print("\t");
      Serial.println(time - lastTime);
    }
  }
}
void walk(){
  // 計測時間をリセット
  reachedTimeZeroDeg[R] = 0;
  reachedTimeZeroDeg[L] = 0;

  startMotor(R, motorSpeed[R]);
  startMotor(L, motorSpeed[L]);

  // 通過するまで
  unsigned long hallWasActive[] ={false,false};
  while(true){
    //motorSlowedTime
    int sides[] = {R,L};
    for (int side : sides) {
      // slowしてから一定時間たっていたら元のスピードに戻す
      if(millis() - motorSlowedTime[side] > 200){
        // Serial.print(side);
        // Serial.println(" normal");
        startMotor(side, MOTOR_SPEED_NORMAL);
      }

      // ホールセンサに反応したら速度計測へ
      bool hallActive = isHallActive(side);
      if(!hallWasActive[side] && hallActive){
        calcSpeed(side);
      }
      hallWasActive[side] = hallActive;
    }


    if(reachedCountZeroDeg[R] > 3 && reachedCountZeroDeg[L] > 3){
      reachedCountZeroDeg[R]=0;
      reachedCountZeroDeg[L]=0;
      break;
    }

    
  }

}

void stop(){
  stopMotor(R);
  stopMotor(L);
}



void loop()
{
  walk();
  stop();
  delay(1000);
}