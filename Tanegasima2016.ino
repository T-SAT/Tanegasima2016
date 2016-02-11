#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include "gyro.h"

#define RXPIN 4
#define TXPIN 5

#define GOAL_FLAT 0721.0760
#define GOAL_FLON 0760.0760

#define GPS_SAMPLING_RATE 10000 //[ms]
#define UNAVAILABLE 0
#define AVAILABLE 1

#define DEG2RAD (PI/180.0)
#define RAD2DEG (180/PI)
#define sign(n) ((n > 0) - (n < 0))

#define GAIN_P 5.0 // 比例ｹﾞｲﾝ
#define GAIN_I 0.0000005 // 積分ｹﾞｲﾝ
#define GAIN_D 0.5 //微分ゲイン

#define Ka 1.0

TinyGPSPlus gps;
SoftwareSerial ss(RXPIN, TXPIN);

typedef struct {
  float OriginFlat;
  float OriginFlon;
  float DestFlat;
  float DestFlon;
} VECTOR;

VECTOR CurrentVector, GoalVector;
int GPSReceiveFlag = UNAVAILABLE;
float GoalAngle;

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
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void ReceiveGPSData(void)
{
  while (ss.available())
    gps.encode(ss.read());
  GPSReceiveFlag = AVAILABLE;
}

float getDt(void)
{
  static long lastTime = 0;

  long nowTime = millis();
  float time = (float)(nowTime - lastTime);
  time = max(time, 20);  //timeは20[us]以上
  time /= 1000;  //[usec] => [sec]
  lastTime = nowTime;

  return ( time );
}

void MotorControl(const int MotorL, const int MotorR)
{
  int motorl, motorr;

  motorl = constrain(MotorL, 0, 255);
  motorr = constrain(MotorR, 0, 255);
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
  currentAngle = GetTransitionAngle(currentAngle, -180, 180);
  goalAngle = GetTransitionAngle(goalAngle, -180, 180);
  angle = currentAngle - goalAngle;
  angle = -angle;
  angle = sign(angle)*180 - angle;
  
  return(angle * DEG2RAD);
}

float PIDctrl(float dCommand, float dVal, float dt)
{
  static float s_dErrIntg = 0;
  static float s_iErrDet = 0;
  static float dRet = 0;
  float dErr, iErr;
  
  // 誤差
  dErr = dCommand - dVal;
  // 誤差積分
  s_dErrIntg += dErr * dt;
  // 誤差微分
  iErr = dErr - s_iErrDet;
  //if(iErr < 0)
  //  iErr = -iErr;  
  s_iErrDet = dErr;
  // 制御入力
  dRet = GAIN_P * dErr + GAIN_I * s_dErrIntg + GAIN_D * iErr;

  //fprintf(stderr, "de:%f\tie:%f\n", dErr, iErr);
  return (dRet);
}

void SteerControl(float Command_rad, float Current_rad, float dt)
{
  float ControlValue;

  ControlValue = PIDctrl(Command_rad, Current_rad, dt);
  if(ControlValue < 0) {
    ControlValue = -ControlValue;
    ControlValue = constrain(ControlValue, 1, 255);
    //fprintf(stderr, "cr:%f\t", ControlValue);
    MotorControl(255, 255 - ControlValue); 
    //MotorControl(255 - ControlValue, 255);
  }
  else{
    ControlValue = constrain(ControlValue, 1, 255);
    //fprintf(stderr, "cl:%f\t", ControlValue);
    MotorControl(255 - ControlValue, 255);
    //MotorControl(255, 255 - ControlValue); 
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ss.begin(9600);
  float goalAngle, currentAngle;

  gelay(1000);
  CurrentVector.OriginFlat = gps.location.lat();
  CurrentVector.OriginFlon = gps.location.lng();
  MotorControl(255, 255);
  delay(5000);
  MotorControl(0, 0);
  CurrentVector.DestFlat = gps.location.lat();
  CurrentVector.DestFlon = gps.location.lng();
  GoalVector.OriginFlat = gps.location.lat();
  GoalVector.OriginFlon = gps.location.lng();
  GoalVector.DestFlat = GOAL_FLAT;
  GoalVector.DestFlon = GOAL_FLON;

  GoalAngle = GetToGoalAngle_rad(CurrentVector, GoalVector);                
  MsTimer2::set(GPS_SAMPLING_RATE, ReceiveGPSData);
  MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly:
  float dt;
  float gx, gy, gz;
  static float angle = 0.0;
  
  dt = getDt();
  measure_gyro(&gx, &gy, &gz);
  angle += gz * dt;
  angle = GetTransitionAngle(angle , -180*DEG2RAD, 180*DEG2RAD);
  if(GPSReceiveFlag == AVAILABLE) {
    CurrentVector.OriginFlat = CurrentVector.DestFlat;
    CurrentVector.OriginFlon = CurrentVector.DestFlon;
    CurrentVector.DestFlat = GoalVector.OriginFlat = gps.location.lat();
    CurrentVector.DestFlon = GoalVector.OriginFlon = gps.location.lng();
    GoalAngle = GetToGoalAngle_rad(CurrentVector, GoalVector);
    angle = 0.0;
  }
  SteerControl(angle * Ka, -GoalAngle, dt);     
}
