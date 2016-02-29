#include <SPI.h>
#include <SD.h>
#include "skLPSxxSPI.h"
#include "L3GD20.h"
#include "Control.h"
#include "Motor.h"
#include "Other.h"
#include "Save.h"
#include "XBEE.h"

#ifdef RUN
float PressureOrigin = 0;
float RC_LPS[2] = {0}; 

void setup()
{
    float angleR, angleG, angleToGoal;
    unsigned long distance;
    float controlValue;
    float data[10];
    float pressureOrigin;
    unsigned long f = millis();

    /*>>>>>>>>>>>>>>>>>> シリアル通信の初期化 <<<<<<<<<<<<<<<<<<<*/
    Serial.begin(9600);
    ss.begin(9600);
    wireless.begin(9600);

    /*>>>>>>>>>>>>>>>>>> 各種センサの初期化 <<<<<<<<<<<<<<<<<<<<*/
    gyro.Init(L3GD20_CSPIN, ODR760BW100);
    gyro.SetFrequencyOfHPF(HPF1);
    lps.PressureInit();
    save.InitSDSlot(SD_CSPIN);

    /*>>>>>>>>>>>>>>>>>> モータの初期化 <<<<<<<<<<<<<<<<<<<<*/
    motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
    motor.SetControlLimit(MIN_PWM_VALUE, MAX_PWM_VALUE);

    lps.PressureRead();
    RC_LPS[0] = lps.GetPressure();
    while((millis() - f) < 10000) {
        lps.PressureRead();
        pressureOrigin = lps.GetPressure();
        RC_LPS[1] = 0.99 * RC_LPS[0] + 0.01 * pressureOrigin;
        pressureOrigin = RC_LPS[1];
        RC_LPS[0] = RC_LPS[1];
    }
    lps.SetOriginPressureValue(pressureOrigin);

    /*>>>>>>>>>>>>>>>>>> 落下検知 <<<<<<<<<<<<<<<<<<<<*/
    unsigned long startTime = millis();

    RC_LPS[0] = 0;
    RC_LPS[1] = 0;
    while(1) {
        GetAllSensorData(data);       
        RC_LPS[1] = 0.9 * RC_LPS[0] + 0.1 * data[LPSxx_H];
        RC_LPS[0] = RC_LPS[1];
        data[SENSOR_NUM + 0] = RC_LPS[1];
        data[SENSOR_NUM + 1] = CheckCurrentState(data[LPSxx_H]);
        wireless.TransferData(data, SENSOR_NUM + 2);
        save.OnSD("LOG2.csv", data, SENSOR_NUM + 2);
        if(data[SENSOR_NUM + 0] == ST_LAND)
            break;
        if(millis() - startTime >= CUT_PARA_TIME)
            break;
    }
        
    ReleaseParachute(PARACHUTEPIN, 1000);

    /*------------- ログ記録 ---------------------*/
    wireless.println("fall success!!");
    save.OnSDStr("LOG.csv", "fall success!!");

    delay(10000); 

    /*>>>>>>>>>>>>>>>>>> 誘導制御（一巡目）<<<<<<<<<<<<<<<<<<<<*/
    gelay(GPS_SAMPLING_RATE);
    CurrentVector.OriginFlat = gps.location.lat();
    CurrentVector.OriginFlon = gps.location.lng();
    
    GetAllSensorData(data);

    distance =
        (unsigned long)TinyGPSPlus::distanceBetween(
                CurrentVector.OriginFlat, CurrentVector.OriginFlon,
                GOAL_FLAT, GOAL_FLON);

    data[SENSOR_NUM + 0] = distance;
    
    /*------------- ログ記録 ---------------------*/
    wireless.TransferData(data, SENSOR_NUM + 1);
    save.OnSD("LOG.csv", data, SENSOR_NUM + 1);

    if(distance <= GOAL_RANGE) {
        wireless.println("goal!");
        Serial.println("goal!");
        motor.Control(0, 0);
        while(1);
    }
    motor.SetPIDGain(VGAIN_P, VGAIN_I, VGAIN_D);
    motor.RunStraight(10000);
    motor.SetPIDGain(AGAIN_P, AGAIN_I, AGAIN_D);
    gelay(GPS_SAMPLING_RATE);

    GetAllSensorData(data);

    CurrentVector.DestFlat = gps.location.lat();
    CurrentVector.DestFlon = gps.location.lng();
    angleR = TinyGPSPlus::courseTo(
            CurrentVector.OriginFlat, CurrentVector.OriginFlon,
            CurrentVector.DestFlat, CurrentVector.DestFlon);
    angleG = TinyGPSPlus::courseTo(
            CurrentVector.DestFlat, CurrentVector.DestFlon,
            GOAL_FLAT, GOAL_FLON);
    angleToGoal = -DEG2RAD * AngleNormalization(angleR - angleG);

    data[SENSOR_NUM + 0] = angleToGoal;

    /*------------- ログ記録 ---------------------*/
    wireless.TransferData(data, SENSOR_NUM + 1);
    save.OnSD("LOG.csv", data, SENSOR_NUM + 1);

    CurrentVector.OriginFlat = CurrentVector.DestFlat;
    CurrentVector.OriginFlon = CurrentVector.DestFlon;
    motor.SteerControl(0, angleToGoal);
}

void loop()
{
    float gx, gy, gz;
    float dt;
    float flat, flon;
    float angle, angleR, angleG, angleToGoal, distance;
    static float currentAngle = 0;
    float error;
    float data[20];

    /*>>>>>>>>>>>>>>>>>> 誘導制御開始（n巡目）<<<<<<<<<<<<<<<<<<<<*/
    dt = getDt();
    gyro.GetPhysicalValue_deg(&gx, &gy, &gz);
    currentAngle += gz * dt;
    currentAngle = DEG2RAD * AngleNormalization(currentAngle);
    gelay(GPS_SAMPLING_RATE);
    angle = 0;
    CurrentVector.DestFlat = gps.location.lat();
    CurrentVector.DestFlon = gps.location.lng();

    distance =
        (unsigned long)TinyGPSPlus::distanceBetween(
                CurrentVector.DestFlat, CurrentVector.DestFlon,
                GOAL_FLAT, GOAL_FLON);

    if(distance <= GOAL_RANGE) {
        Serial.println("goal!");
        motor.Control(0, 0);
        while(1);
    }

    angleR = TinyGPSPlus::courseTo(
            CurrentVector.OriginFlat, CurrentVector.OriginFlon,
            CurrentVector.DestFlat, CurrentVector.DestFlon);

    angleG = TinyGPSPlus::courseTo(
            CurrentVector.OriginFlat, CurrentVector.OriginFlon,
            GOAL_FLAT, GOAL_FLON);

    angleToGoal = -DEG2RAD * AngleNormalization(angleR - angleG);
    error = angleToGoal - currentAngle;
    motor.SteerControl(0, error);
    CurrentVector.OriginFlat = CurrentVector.DestFlat;
    CurrentVector.OriginFlon = CurrentVector.DestFlon;

    GetAllSensorData(data);
    data[SENSOR_NUM + 0] = distance;
    data[SENSOR_NUM + 1] = error;

    /*------------- ログ記録 ---------------------*/
    wireless.TransferData(data, SENSOR_NUM + 2);
    save.OnSD("LOG.csv", data, SENSOR_NUM + 2);
}
#endif

#ifdef TEST
float PressureOrigin;

void setup() {
    Serial.begin(9600);
    ss.begin(9600);
    wireless.begin(9600);

    gyro.Init(L3GD20_CSPIN, ODR760BW100);
    gyro.SetFrequencyOfHPF(HPF1);
    lps.PressureInit();
    save.InitSDSlot(SD_CSPIN);
    motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
    motor.SetPIDGain(VGAIN_P, VGAIN_I, VGAIN_D);
    motor.SetControlLimit(MIN_PWM_VALUE, MAX_PWM_VALUE);
    lps.PressureRead();
    PressureOrigin = lps.GetPressure();
    lps.SetOriginPressureValue(PressureOrigin);
}

void loop() {
    float gx, gy, gz;
    float data[8];

    lps.PressureRead();
    data[0] = lps.GetPressure();
    data[1] = lps.GetTempreture();
    data[2] = lps.GetAltitude(data[0]);
    gyro.GetPhysicalValue_deg(&gx, &gy, &gz);
    data[3] = gx;
    data[4] = gy;
    data[5] = gz; 
    data[6] = gps.location.lat();
    data[7] = gps.location.lng();
    wireless.TransferData(data, 8);
    save.OnSD("panna.csv", data, 8);
}
#endif
