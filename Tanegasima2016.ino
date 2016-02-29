#include <SPI.h>
#include <SD.h>
#include "HC_SR04.h"
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
    float data[SENSOR_NUM + 2];
    unsigned long f = millis();

    /*>>>>>>>>>>>>>>>>>> シリアル通信の初期化 <<<<<<<<<<<<<<<<<<<*/
    Serial.begin(9600);
    ss.begin(9600);
    wireless.begin(9600);

    /*>>>>>>>>>>>>>>>>>> 各種センサの初期化 <<<<<<<<<<<<<<<<<<<<*/
    gyro.Init(L3GD20_CSPIN, ODR760BW100);
    gyro.SetFrequencyOfHPF(HPF1);
    lps.PressureInit();
    sonic.Init(0, 1, 10);
    save.InitSDSlot(SD_CSPIN);

    /*>>>>>>>>>>>>>>>>>> モータの初期化 <<<<<<<<<<<<<<<<<<<<*/
    motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
    motor.SetControlLimit(MIN_PWM_VALUE, MAX_PWM_VALUE);

    lps.PressureRead();
    lps.SetOriginPressureValue(lps.GetPressure());

    /*>>>>>>>>>>>>>>>>>> 落下検知 <<<<<<<<<<<<<<<<<<<<*/
    unsigned long startTime = millis();

    RC_LPS[0] = 0;
    RC_LPS[1] = 0;
    while(1) {
        GetAllSensorData(data);       
        RC_LPS[1] = 0.9 * RC_LPS[0] + 0.1 * data[LPSxx_H];
        RC_LPS[0] = RC_LPS[1];

        if(CheckCurrentState(RC_LPS[1]) == ST_DOWN && sonic.ReadLength() != 0.0)
            RC_LPS[1] = (RC_LPS[1] + sonic.ReadLength()) / 2.0;

        data[SENSOR_NUM + 0] = RC_LPS[1];
        data[SENSOR_NUM + 1] = CheckCurrentState(RC_LPS[1]);

        /*------------- ログ記録 ---------------------*/
        wireless.TransferData(data, SENSOR_NUM + 2);
        save.OnSD("LOG.csv", data, SENSOR_NUM + 2);

        if(data[SENSOR_NUM + 1] == ST_LAND)
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
    OriginFlat = gps.location.lat();
    OriginFlon = gps.location.lng();

    GetAllSensorData(data);

    /*------------- ログ記録 ---------------------*/
    wireless.TransferData(data, SENSOR_NUM);
    save.OnSD("LOG.csv", data, SENSOR_NUM);

    //motor.SetPIDGain(VGAIN_P, AGAIN_I, AGAIN_D);
    //motor.RunStraight(10000);
    motor.Control(255, 255);
    delay(10000);
    motor.Control(0, 0);
    motor.SetPIDGain(AGAIN_P, AGAIN_I, AGAIN_D);
    gelay(GPS_SAMPLING_RATE);

    GetAllSensorData(data);

    DestFlat = gps.location.lat();
    DestFlon = gps.location.lng();
    angleR = TinyGPSPlus::courseTo(
            OriginFlat,  OriginFlon,
            DestFlat,  DestFlon);
    angleG = TinyGPSPlus::courseTo(
            DestFlat,  DestFlon,
            GOAL_FLAT, GOAL_FLON);
    angleToGoal = -DEG2RAD * AngleNormalization(angleR - angleG);

    OriginFlat =  DestFlat;
    OriginFlon =  DestFlon;
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
    float data[SENSOR_NUM + 2];

    /*>>>>>>>>>>>>>>>>>> 誘導制御開始（n巡目）<<<<<<<<<<<<<<<<<<<<*/
    dt = getDt();
    gyro.GetPhysicalValue_deg(&gx, &gy, &gz);
    currentAngle += gz * dt;
    currentAngle = DEG2RAD * AngleNormalization(currentAngle);
    gelay(GPS_SAMPLING_RATE);
    angle = 0;
    DestFlat = gps.location.lat();
    DestFlon = gps.location.lng();

    distance =
        (unsigned long)TinyGPSPlus::distanceBetween(
                DestFlat,  DestFlon,
                GOAL_FLAT, GOAL_FLON);

    if(distance <= GOAL_RANGE) {
        wireless.println("goal!");
        save.OnSDStr("LOG.csv", "goal!");
        motor.Control(0, 0);
        while(1) {
            GetAllSensorData(data);

            /*------------- ログ記録 ---------------------*/
            wireless.TransferData(data, SENSOR_NUM);
            save.OnSD("LOG.csv", data, SENSOR_NUM);
        }
    }

    angleR = TinyGPSPlus::courseTo(
            OriginFlat,  OriginFlon,
            DestFlat,  DestFlon);

    angleG = TinyGPSPlus::courseTo(
            OriginFlat,  OriginFlon,
            GOAL_FLAT, GOAL_FLON);

    angleToGoal = -DEG2RAD * AngleNormalization(angleR - angleG);
    error = angleToGoal - currentAngle;
    motor.SteerControl(0, error);
    OriginFlat =  DestFlat;
    OriginFlon =  DestFlon;

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
    sonic.Init(0, 1, 10);
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
    data[6] = sonic.ReadLength();
    data[7] = sonic.ReadLength();
    wireless.TransferData(data, 8);
    save.OnSD("panna.csv", data, 8);
}
#endif
