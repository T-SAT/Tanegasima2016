#include "Other.h"
#include "L3GD20.h"
#include "Save.h"

/*>>>>>>>>>>>>>>>>>>> 各種変数・型定義 <<<<<<<<<<<<<<<<<<*/
float OriginFlat, OriginFlon, DestFlat, DestFlon;
int isGPSAvailable = UNAVAILABLE;

/*>>>>>>>>>>>>>>>>>>> コンストラクタ <<<<<<<<<<<<<<<<<<*/
TinyGPSPlus gps;
SoftwareSerial ss(RXPIN_GPS, TXPIN_GPS);
XBEE wireless(RXPIN_XBEE, TXPIN_XBEE);
skLPSxxx lps(LPS331AP, LPS331AP_CSPIN);

/*>>>>>>>>>>>>>>>>>>> 関数定義 <<<<<<<<<<<<<<<<<<*/
void gelay(unsigned long ms)
{
    unsigned long start = millis();
    do {
        while (ss.available())
            gps.encode(ss.read());
    } while (millis() - start < ms);
}

int ReceiveGPSDatanormal(void)
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

float AngleNormalization(float angle)
{
    if(angle>=180)
        angle =  angle - 360;
    else if(angle<=-180)
        angle = angle + 360;

    return(angle);
}

int CheckCurrentState(float altitude)
{
    static int cur_st = ST_START;

    if (altitude >= UP_ALTITUDE && cur_st == ST_START) {
        cur_st = ST_UP;
        return (cur_st);
    }
    else if (altitude >= TOP_ALTITUDE && cur_st == ST_UP) {
        cur_st = ST_TOP;
        return(cur_st);
    }
    else if (altitude <= DOWN_ALTITUDE && cur_st == ST_TOP) {
        cur_st = ST_DOWN;
        return (cur_st);
    }
    else if (altitude <= LAND_ALTITUDE && cur_st == ST_DOWN) { 
        cur_st = ST_LAND;
        return (cur_st);
    }
    else {
        return(cur_st);
    }

}

void ReleaseParachute(int parapin, unsigned long heatTime)
{
    pinMode(parapin, OUTPUT);
    digitalWrite(parapin, HIGH);
    delay(heatTime);
    digitalWrite(parapin, LOW);
}

void GetAllSensorData(float *data)
{
    float t, p, h;

    t = lps.GetTempreture();
    p = lps.GetPressure();
    h = lps.GetAltitude(p);
    data[LPSxx_T] = t;
    data[LPSxx_P] = p;
    data[LPSxx_H] = h;

    data[GPS_LAT] = gps.location.lat();
    data[GPS_LON] = gps.location.lng();
}

