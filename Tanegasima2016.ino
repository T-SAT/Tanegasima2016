#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include "L3GD20.h"
#include "Control.h"
#include "Motor.h"

#define TEST
//#define RUN

//#define NORMAL_GPS_DATA
#define AVERAGE_GPS_DATA

#define RXPIN 4
#define TXPIN 5

#define GOAL_FLAT 35.515935
#define GOAL_FLON 134.173115

#define GPS_SAMPLING_RATE 10000 //[ms]
#define GPS_SAMPLING_NUM 10
#define UNAVAILABLE 0
#define AVAILABLE 1

#define GAIN_P 8.0 // 比例ｹﾞｲﾝ
#define GAIN_I 0.05 // 積分ｹﾞｲﾝ
#define GAIN_D 0.8 //微分ゲイン

#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)
#define AngleNormalization(n) if(n > 180) n -= 360; else if(n < -180) n += 360;
#define sign(n) ((n > 0) - (n < 0))

#define RFPin 10
#define RBPin 6
#define LFPin 5
#define LBPin 9

#define MIN_PWM_VALUE 1
#define MAX_PWM_VALUE 255

#define Ka 1.0

#define L3GD20_CS A1

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

  PORTD &= ~(1 << 5);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A4, HIGH);
  motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
  motor.SetControlLimit(1, 255);
  gyro.Init(L3GD20_CS, ODR760BW100);
  gyro.SetFrequencyOfHPF(HPF10);
}

void loop()
{
  float x, y, z;
  float angle1, angle2, angle3, angle4;
  static float angle = 0;
  float dt;
  float flat0 = 35.515901, flon0 = 134.173071;
  float flat1 = 35.516046, flon1 = 134.172875;
  float flat2 = 35.516066, flon2 = 134.173251;
  float flat3 = 35.515759, flon3 = 134.172881;
  float flat4 = 35.515757, flon4 = 134.173283;
  VECTOR current, goal;

  //motor.Control(100, 100);
  dt = getDt();
  gyro.GetPhysicalValue_deg(&x, &y, &z);
  angle += z * dt;

  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(z);
  Serial.print("\t");
  Serial.print(angle);
  Serial.println();
  delay(10);

}
#endif

#ifdef RUN
void setup() {
  // put your setup code here, to run once:
  float goalAngle, currentAngle;
  float flatAve = 0, flonAve = 0;
  int i;

  Serial.begin(9600);
  ss.begin(9600);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A4, HIGH);

  motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
  motor.SetPIDGain(GAIN_P, GAIN_I, GAIN_D);
  motor.SetControlLimit(MIN_PWM_VALUE, MAX_PWM_VALUE);
  gyro.Init(L3GD20_CS, ODR760BW100);
  gyro.SetFrequencyOfHPF(HPF1);
  control.SetPIDGain(GAIN_P, GAIN_I, GAIN_D);
  for (i = 0; i < GPS_SAMPLING_NUM; i++) {
    gelay(1000);
    flatAve += gps.location.lat();
    flonAve += gps.location.lng();
  }
  CurrentVector.OriginFlat = flatAve / GPS_SAMPLING_NUM;
  CurrentVector.OriginFlon = flonAve / GPS_SAMPLING_NUM;
  motor.Control(255, 255);
  gelay(5000);
  motor.Control(0, 0);
  CurrentVector.DestFlat = gps.location.lat();
  CurrentVector.DestFlon = gps.location.lng();
  GoalVector.OriginFlat = gps.location.lat();
  GoalVector.OriginFlon = gps.location.lng();
  GoalVector.DestFlat = GOAL_FLAT;
  GoalVector.DestFlon = GOAL_FLON;
  GoalAngle = GetToGoalAngle_rad(CurrentVector, GoalVector);
#ifdef NORMAL_GPS_DATA
  MsTimer2::set(GPS_SAMPLING_RATE, ReceiveGPSDataAve);
#endif

#ifdef AVERAGE_GPS_DATA
  MsTimer2::set(GPS_SAMPLING_RATE, ReceiveGPSData);
#endif
  MsTimer2::start();
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

  motor.SteerControl(GoalAngle, angle_rad * Ka);
}
#endif
