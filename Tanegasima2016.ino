#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>

#define RXPIN 4
#define TXPIN 5

#define GOAL_FLAT 0721.0760
#define GOAL_FLON 0760.0760

#define GPS_SAMPLING_RATE 10000 //[ms]
#define UNAVAILABLE 0
#define AVAILABLE 1

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
  while(ss.available()) 
    gps.encode(ss.read());  
  GPSReceiveFlag = AVAILABLE;
}

float getDt(void)
{
  static long lastTime=0;

  long nowTime = millis();
  float time = (float)(nowTime - lastTime);
  time = max(time, 20);  //timeは20[us]以上
  time /= 1000;  //[usec] => [sec]
  lastTime = nowTime;

  return( time );
}

void MotorControl(const int MotorL, const int MotorR)
{
    int motorl, motorr;

    motorl = constrain(MotorL, 0, 255);
    motorr = constrain(MotorR, 0, 255);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ss.begin(9600);

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
  
  MsTimer2::set(GPS_SAMPLING_RATE, ReceiveGPSData);
  MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly:
  float dt;

  dt = getDt();
  
}
