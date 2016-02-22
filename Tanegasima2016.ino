#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include "L3GD20.h"
#include "Control.h"
#include "Motor.h"
#include "GPS.h"

#define TEST
//#define RUN

#define NORMAL_GPS_DATA
//#define AVERAGE_GPS_DATA

#define RXPIN 4
#define TXPIN 3

#define GOAL_FLAT 35.515819
#define GOAL_FLON 134.171813

#define GPS_SAMPLING_RATE 1000 //[ms]
#define GPS_SAMPLING_NUM 10
#define UNAVAILABLE 0
#define AVAILABLE 1

#define VGAIN_P 0.5 // 比例ｹﾞｲﾝ
#define VGAIN_I 0.0 // 積分ｹﾞｲﾝ
#define VGAIN_D 0.0 //微分ゲイン

#define AGAIN_P 12.0
#define AGAIN_I 0.05
#define AGAIN_D 0.5

#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)
#define AngleNormalization(n) if(n > 180) n -= 360; else if(n < -180) n += 360;
#define sign(n) ((n > 0) - (n < 0))

#define RBPin 6
#define RFPin 10
#define LBPin 5
#define LFPin 9

#define MIN_PWM_VALUE 1
#define MAX_PWM_VALUE 127

#define L3GD20_CS A2

GPS gpso(RXPIN, TXPIN);

TinyGPSPlus gps;
SoftwareSerial ss(RXPIN, TXPIN);

typedef struct {
  float OriginFlat;
  float OriginFlon;
  float DestFlat;
  float DestFlon;
} VECTOR;

VECTOR CurrentVector, GoalVector;
int  isGPSAvailable = UNAVAILABLE;
float GoalAngle;
float Flat, Flon;

float GetTransitionAngle(float FormerAngle, float MinValue, float MaxValue)
{
  float mod;
  float AfterAngle;

  if (FormerAngle < MinValue) {
    mod = fmod(FormerAngle, fabs(MinValue));
    AfterAngle = MaxValue + mod;
  } else if (FormerAngle > MaxValue) {
    mod = fmod(FormerAngle, fabs(MaxValue));
    AfterAngle = MinValue + mod;
  } else {
    AfterAngle = FormerAngle;
  }

  return (AfterAngle);
}

static void gelay(unsigned long ms)
{
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void ReceiveGPSData(void)
{
  while (ss.available())
    gps.encode(ss.read());
}

int ReceiveGPSDataNormal(void)
{
  static unsigned int ReceiveGPSNum = 0;
  static unsigned long lastTime = millis();
  unsigned long nowTime = millis();

  ReceiveGPSNum += ss.available();
  while (ss.available())
    gps.encode(ss.read());
  if (ReceiveGPSNum >= 70) {
    while (ss.available())
      ss.read();
    ReceiveGPSNum = 0;
    if ((nowTime - lastTime) >= GPS_SAMPLING_RATE) {
      lastTime = millis();
      return (AVAILABLE);
    }
  }
  return (UNAVAILABLE);
}

void ReceiveGPSDataAve(void)
{
  static int receiveNum = 0;
  static float flatAve = 0, flonAve = 0;

  ReceiveGPSData();
  flatAve += gps.location.lat();
  flonAve += gps.location.lng();
  receiveNum++;
  if (receiveNum >= GPS_SAMPLING_NUM) {
    Flat = flatAve / receiveNum;
    Flon = flonAve / receiveNum;
    isGPSAvailable = AVAILABLE;
    receiveNum = 0.0;
    flatAve = 0.0;
    flonAve = 0.0;
  }
}

void RunPeriodicallyMs(int (*f)(void), unsigned long period)
{
  static unsigned long lastTime = millis();
  unsigned long nowTime = millis();
  unsigned long time = nowTime - lastTime;
  static int isContinue = 1;
  if (time >= period || isContinue) {
    isContinue = f();
    lastTime = millis();
  } else
					return;
}

float getDt(void)
{
  static long lastTime = millis();

  long nowTime = millis();
  float time = (float)(nowTime - lastTime);
  time = max(time, 20);  //timeは20[us]以上
  time /= 1000;  //[usec] => [sec]
  lastTime = nowTime;

  return ( time );
}

float GetToGoalAngle_rad(VECTOR current, VECTOR goal)
{
  float currentAngle, goalAngle;
  float angle;

  currentAngle = TinyGPSPlus::courseTo(
                   current.DestFlat, current.DestFlon,
                   current.OriginFlat, current.OriginFlon);
  goalAngle = TinyGPSPlus::courseTo(
                goal.OriginFlat, goal.OriginFlon,
                goal.DestFlat, goal.DestFlon);
  AngleNormalization(currentAngle);
  AngleNormalization(goalAngle);

  angle = goalAngle - currentAngle;
  angle = sign(angle) * 180 - angle;

  return (angle * DEG2RAD);
}

#ifdef TEST
void setup()
{
  Serial.begin(9600);
  ss.begin(4800);
 
  PORTD &= ~(1 << 5);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A4, HIGH);
  motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
  motor.SetPIDGain(VGAIN_P, VGAIN_I, VGAIN_D);
  motor.SetControlLimit(0, 255);
  //gyro.Init(L3GD20_CS, ODR760BW100);
  //gyro.SetFrequencyOfHPF(HPF1);
  while (ss.available())
    ss.read();
}

void loop()
{
  float x, y, z;
  static float angle = 0;
  float dt;

  motor.Control(255, 255);
  delay(10000);
  motor.Control(0, 0);
  delay(10000);
  /*Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\t");
    Serial.print(angle);
    Serial.print("\t");
    Serial.println();
  */

  delay(10);
  /**/
}
#endif

#ifdef RUN
void setup() {
  // put your setup code here, to run once:
  float goalAngle, currentAngle;
  float flatAve = 0, flonAve = 0;
  int i;

  Serial.begin(9600);
  ss.begin(4800);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A4, HIGH);
  motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
  motor.SetPIDGain(VGAIN_P, VGAIN_I, VGAIN_D);
  motor.SetControlLimit(MIN_PWM_VALUE, MAX_PWM_VALUE);
  gyro.Init(L3GD20_CS, ODR760BW100);
  gyro.SetFrequencyOfHPF(HPF1);
  do {
    gelay(1000);
  } while (gps.location.lat() == 0 && gps.location.lng() == 0);

  for (i = 0; i < GPS_SAMPLING_NUM; i++) {
    gelay(1000);
    flatAve += gps.location.lat();
    flonAve += gps.location.lng();
  }
  CurrentVector.OriginFlat = flatAve / GPS_SAMPLING_NUM;
  CurrentVector.OriginFlon = flonAve / GPS_SAMPLING_NUM;
  Serial.println("check");
  motor.RunStraight(10000);
  motor.SetPIDGain(AGAIN_P, AGAIN_I, AGAIN_D);
  gelay(1000);
  CurrentVector.DestFlat = gps.location.lat();
  CurrentVector.DestFlon = gps.location.lng();
  GoalVector.OriginFlat = gps.location.lat();
  GoalVector.OriginFlon = gps.location.lng();
  GoalVector.DestFlat = GOAL_FLAT;
  GoalVector.DestFlon = GOAL_FLON;
  GoalAngle = GetToGoalAngle_rad(CurrentVector, GoalVector);
}

void loop() {
  // put your main code here, to run repeatedly:
  float dt;
  float gx, gy, gz;
  static float angle_rad = 0.0;

  dt = getDt();
  gyro.GetPhysicalValue_deg(&gx, &gy, &gz);
  angle_rad += DEG2RAD * gz * dt;
  angle_rad = GetTransitionAngle(angle_rad , -180 * DEG2RAD, 180 * DEG2RAD);
#ifdef NORMAL_GPS_DATA
  gelay(1000);
  isGPSAvailable = AVAILABLE;
  if (isGPSAvailable) {
    CurrentVector.OriginFlat = CurrentVector.DestFlat;
    CurrentVector.OriginFlon = CurrentVector.DestFlon;
    CurrentVector.DestFlat = GoalVector.OriginFlat = gps.location.lat();
    CurrentVector.DestFlon = GoalVector.OriginFlon = gps.location.lng();
    if ((unsigned long)TinyGPSPlus::distanceBetween(
          GoalVector.OriginFlat, GoalVector.OriginFlon,
          GoalVector.DestFlat, GoalVector.DestFlon) <= 1.0) {
      motor.Control(0, 0);
      while (1) {
        delay(1000);
      }
    }
    GoalAngle = GetToGoalAngle_rad(CurrentVector, GoalVector);
    angle_rad = 0.0;
    isGPSAvailable = UNAVAILABLE;
  }
#endif

#ifdef AVERAGE_GPS_DATA
  if ( isGPSAvailable) {
    CurrentVector.OriginFlat = CurrentVector.DestFlat;
    CurrentVector.OriginFlon = CurrentVector.DestFlon;
    CurrentVector.DestFlat = GoalVector.OriginFlat = Flat;
    CurrentVector.DestFlon = GoalVector.OriginFlon = Flon;
    if ((unsigned long)TinyGPSPlus::distanceBetween(
          GoalVector.OriginFlat, GoalVector.OriginFlon,
          GoalVector.DestFlat, GoalVector.DestFlon) <= 1.0) {
      motor.Control(0, 0);
      while (1) {
        delay(1000);
      }
    }
    GoalAngle = GetToGoalAngle_rad(CurrentVector, GoalVector);
    angle_rad = 0.0;
    isGPSAvailable = UNAVAILABLE;
  }
#endif
  motor.SteerControl(GoalAngle, angle_rad);
  Serial.print("GoalAngle="); 
  Serial.print("\t");
  Serial.print(RAD2DEG*GoalAngle);
  Serial.print("\t");
  Serial.print(RAD2DEG*angle_rad);
  Serial.print("\t");
  Serial.println(gz);
  
}
#endif
