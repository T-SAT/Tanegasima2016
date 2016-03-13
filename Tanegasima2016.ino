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
    //wireless.begin(9600);

    /*>>>>>>>>>>>>>>>>>> モータの初期化 <<<<<<<<<<<<<<<<<<<<*/
    motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
    motor.SetControlLimit(MIN_PWM_VALUE, MAX_PWM_VALUE);

    /*>>>>>>>>>>>>>>>>>> 各種センサの初期化 <<<<<<<<<<<<<<<<<<<<*/
    gyro.Init(L3GD20_CSPIN, ODR760BW100);
    gyro.SetFrequencyOfHPF(HPF1);
    lps.PressureInit();
    //sonic.Init(0, 1, 10);
    save.InitSDSlot(SD_CSPIN);

    lps.PressureRead();
    lps.SetOriginPressureValue(lps.GetPressure());
    /*>>>>>>>>>>>>>>>>>> 落下検知 <<<<<<<<<<<<<<<<<<<<*/
    unsigned long startTime = millis();

    save.OnSDStr("LOG.csv", "start!!");
    save.OnSDStr("LOG.csv", "#T,#P,#H,#LAT,#LON,#RCH,#STATE");
    RC_LPS[0] = 0;
    RC_LPS[1] = 0;
    while(1) {
        lps.PressureRead();
        GetAllSensorData(data);       
        RC_LPS[1] = 0.9 * RC_LPS[0] + 0.1 * data[LPSxx_H];
        RC_LPS[0] = RC_LPS[1];

        if(CheckCurrentState(RC_LPS[1]) == ST_DOWN && sonic.ReadLength() != 0.0)
            RC_LPS[1] = (RC_LPS[1] + sonic.ReadLength()) / 2.0;
        data[SENSOR_NUM + 0] = RC_LPS[1];
        data[SENSOR_NUM + 1] = CheckCurrentState(RC_LPS[1]);

        /*------------- ログ記録 ---------------------*/
        //wireless.TransferData(data, SENSOR_NUM + 2);
        save.OnSD("LOG.csv", data, SENSOR_NUM + 2);

        if(data[SENSOR_NUM + 1] == ST_LAND)
            break;
        if(millis() - startTime >= CUT_PARA_TIME)
            break;
    }
    ReleaseParachute(PARACHUTEPIN, 2000);

    /*------------- ログ記録 ---------------------*/
    //wireless.println("fall success!!");
    save.OnSDStr("LOG.csv", "fall success!!");

    delay(10000); 

    /*>>>>>>>>>>>>>>>>>> 誘導制御（一巡目）<<<<<<<<<<<<<<<<<<<<*/
    gelay(GPS_SAMPLING_RATE);
    OriginFlat = gps.location.lat();
    OriginFlon = gps.location.lng();
    Serial.print(OriginFlat);
    Serial.print("\t");
    Serial.println(OriginFlon);
    GetAllSensorData(data);

    /*------------- ログ記録 ---------------------*/
    //wireless.TransferData(data, SENSOR_NUM);
    save.OnSDStr("LOG.csv", "#T,#P,#H,#LAT,#LON");
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
    Serial.print(DestFlat);
    Serial.print("\t");
    Serial.println(DestFlon);
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
    save.OnSDStr("LOG.csv", "#T,#P,#H,#LAT,#LON,#DIS,#ERR");
}

void loop()
{
    float gx, gy, gz;
    float dt;
    float flat, flon;
    float angleR, angleG, angleToGoal, distance;
    static float currentAngle = 0;
    float error;
    float data[SENSOR_NUM + 3];

    /*>>>>>>>>>>>>>>>>>> 誘導制御開始（n巡目）<<<<<<<<<<<<<<<<<<<<*/
    dt = getDt();
    gelay(GPS_SAMPLING_RATE);
    DestFlat = gps.location.lat();
    DestFlon = gps.location.lng();

    distance =
        (unsigned long)TinyGPSPlus::distanceBetween(
                DestFlat,  DestFlon,
                GOAL_FLAT, GOAL_FLON);

    if(distance <= GOAL_RANGE) {
        //wireless.println("goal!");
        save.OnSDStr("LOG.csv", "goal!");
        save.OnSDStr("LOG.csv", "#LAT,#LON,#DIS,#ERR");
        motor.Control(0, 0);
        while(1) {
            GetAllSensorData(data);

            /*------------- ログ記録 ---------------------*/
            // wireless.TransferData(data, SENSOR_NUM);
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
    error = angleToGoal;
    motor.SteerControl(0, error);
    OriginFlat =  DestFlat;
    OriginFlon =  DestFlon;
    GetAllSensorData(data);
    data[0] = gps.location.lat();
    data[1] = gps.location.lng();
    data[2] = distance;
    data[3] = error;
    Serial.print(distance); 
    Serial.print("\t");
    Serial.println(error);

    /*------------- ログ記録 ---------------------*/
    //wireless.TransferData(data, SENSOR_NUM + 2);
    save.OnSD("LOG.csv", data, 4);
}
#endif

#ifdef TEST
float PressureOrigin;

void setup() {
    Serial.begin(9600);
    ss.begin(9600);
   // wireless.begin(9600);
    motor.SetPinNum(LFPin, LBPin, RFPin, RBPin);
    motor.SetPIDGain(VGAIN_P, VGAIN_I, VGAIN_D);
    motor.SetControlLimit(MIN_PWM_VALUE, MAX_PWM_VALUE);
}

void loop() {
    motor.Control(-100, 0);
}

#endif
