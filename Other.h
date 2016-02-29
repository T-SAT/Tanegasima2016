#ifndef ETC_H_INCLUDED
#define ETC_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "XBEE.h"
#include "skLPSxxSPI.h"

/*>>>>>>>>>>>>>>>>>>> モードセレクト <<<<<<<<<<<<<<<<<<*/
/*
 * 動作させるモードの行をコメントアウトして使用してください
 */
//#define TEST       //テストモード
#define RUN      //誘導走行モード

/*>>>>>>>>>>>>>>>>>>> 落下落下検知関係<<<<<<<<<<<<<<<<<<*/
/*
 * 落下時の機体の状態を表す定数
 */
#define ST_START  1
#define ST_UP     2
#define ST_TOP    3
#define ST_DOWN   4
#define ST_LAND   5
/*
 * 状態遷移のトリガとなる高度[ms]
 */
#define UP_ALTITUDE     25
#define DOWN_ALTITUDE   10
#define TOP_ALTITUDE    40
#define LAND_ALTITUDE   1
/*
 * パラシュートの切り離しピン 
 */
#define PARACHUTEPIN        8 
/*
 * パラシュートの強制切り離し時間 [ms]
 */
#define CUT_PARA_TIME 300000

/*>>>>>>>>>>>>>>>>>>> 各種センサーピン <<<<<<<<<<<<<<<<<<*/
#define RXPIN_GPS      4 //GPSモジュールのソフトウェアシリアルrxピン
#define TXPIN_GPS      3 //GPSモジュールのソフトウェアシリアルtxピン

#define RXPIN_XBEE     7 //XBEEモジュールのソフトウェアシリアルrxピン
#define TXPIN_XBEE     2 //XBEEモジュールのソフトウェアシリアルtxピン

#define RBPin          6 //HIGHにすると右モータが後転するピン
#define RFPin          1 //HIGHにすると右モータが正転するピン
#define LBPin          5 //HIGHにすると左モータが後転するピン
#define LFPin          9 //HIGHにすると左モータが正転するピン

#define LPS331AP_CSPIN      A1 //LPS331AP(気圧センサ)のCSピン
#define L3GD20_CSPIN        A2 //L3GD20(ジャイロセンサ)のCSピン
#define SD_CSPIN            A3 //SDカードスロットのCSピン
#define ADXL345_CSPIN       A4 //ADXL345(加速度センサ)のCSピン

/*>>>>>>>>>>>>>>>>>>> 座標関係 <<<<<<<<<<<<<<<<<<*/
#define GOAL_FLAT          35.515819  //ゴール緯度[deg]
#define GOAL_FLON          134.171813 //ゴール軽度[deg]
#define GPS_SAMPLING_RATE  1000       //GPSのサンプリング周期[ms]
#define GOAL_RANGE         1.0        //ゴール判定半径[m]

#define GPS_SAMPLING_NUM   10         //平均取る時のGPSデータの取得数
/* GPSデータ利用可能フラグ(ReceiveGPSDataの返り値)*/
#define UNAVAILABLE        0
/* GPSデータ利用不能フラグ(ReceiveGPSDataの返り値)*/
#define AVAILABLE          1

/*>>>>>>>>>>>>>>>>>>> 制御定数 <<<<<<<<<<<<<<<<<<*/
#define VGAIN_P 0.5   //パラメータが角速度の時の比例ｹﾞｲﾝ
#define VGAIN_I 0.0   //パラメータが角速度の時の積分ｹﾞｲﾝ
#define VGAIN_D 0.0   //パラメータが角速度の時の微分ゲイン

#define AGAIN_P 12.0  //パラメータが角度の時の比例ｹﾞｲﾝ
#define AGAIN_I 0.05  //パラメータが角度の時の積分ｹﾞｲﾝ
#define AGAIN_D 0.5   //パラメータが角度の時の微分ゲイ 

#define MIN_PWM_VALUE 1   //モータに与えるpwmduty比の最小値
#define MAX_PWM_VALUE 127 //モータに与えるpwmduty比の最大値

/*>>>>>>>>>>>>>>>>>>> マクロ <<<<<<<<<<<<<<<<<<*/
#define DEG2RAD (PI/180.0)           //度数法から弧度法への数値変換
#define RAD2DEG (180.0/PI)           //弧度法から度数法への数値変換
#define sign(n) ((n > 0) - (n < 0))  //数値の符号判定

/*>>>>>>>>>>>>>>>>>>> 各種変数・型定義 <<<<<<<<<<<<<<<<<<*/
typedef struct {
    float OriginFlat;
    float OriginFlon;
    float DestFlat;
    float DestFlon;
} VECTOR;

typedef enum {
    L3GD20_X = 0,
    L3GD20_Y,
    L3GD20_Z,
    LPSxx_T,
    LPSxx_P,
    LPSxx_H,
    GPS_LAT,
    GPS_LON,
    SENSOR_NUM,
} SENSOR_ID;

extern VECTOR CurrentVector;
extern int isGPSAvailable;

/*>>>>>>>>>>>>>>>>>>> コンストラクタ <<<<<<<<<<<<<<<<<<*/
extern TinyGPSPlus gps;
extern SoftwareSerial ss;
extern XBEE wireless;
extern skLPSxxx lps;

/*>>>>>>>>>>>>>>>>>>> プロトタイプ宣言 <<<<<<<<<<<<<<<<<<*/
void gelay(unsigned long ms);
int ReceiveGPSDatanormal(void);
void RunPeriodicallyMs(int (*f)(void), unsigned long period);
float getDt(void);
float AngleNormalization(float angle);
int CheckCurrentState(float altitude);
void ReleaseParachute(int parapin, unsigned long heatTime);
void GetAllSensorData(float *data);

#endif
