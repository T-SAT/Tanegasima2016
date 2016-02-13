#ifndef L3GD20_H_INCLUDED
#define L3GD20_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

typedef enum {
  ODR95BW12_5 = 0,
  ODR95BW25 = 1,
  ODR190BW12_5 = 4,
  ODR190BW25 = 5,
  ODR190BW50 = 6,
  ODR190BW70 = 7,
  ODR380BW20 = 8,
  ODR380BW25 = 9,
  ODR380BW50 = 10,
  ODR380BW100 = 11,
  ODR760BW30 = 12,
  ODR760BW35 = 13,
  ODR760BW50 = 14,
  ODR760BW100 = 15,
} DRBW;

typedef enum {
  HPF1 = 0,
  HPF2 = 1,
  HPF3 = 2,
  HPF4 = 3,
  HPF5 = 4,
  HPF6 = 5,
  HPF7 = 6,
  HPF8 = 7,
  HPF9 = 8,
  HPF10 = 9,
} HPF;

class L3GD20 {
  public:
    void Init(int CS1, DRBW settingDRBW);
    void SetFrequencyOfHPF(byte frequecy);
    void GetPhysicalValue_deg(float *x, float *y, float *z);
    void GetRowValue(short *x, short *y, short *z);
    void L3GD20_write(byte reg, byte val);
    byte L3GD20_read(byte reg);

  private:
    int L3GD20_CS = SS;
    const int OFFSET_NUM = 1000;
    short offsetxGyro, offsetyGyro, offsetzGyro;

  private:
    const byte L3GD20_WHOAMI = 0x0f;
    const byte L3GD20_CTRL1 = 0x20;
    const byte L3GD20_CTRL2 = 0x21;
    const byte L3GD20_CTRL3 = 0x22;
    const byte L3GD20_CTRL4 = 0x23;
    const byte L3GD20_CTRL5 = 0x24;
    const byte L3GD20_X_L = 0x28;
    const byte L3GD20_X_H = 0x29;
    const byte L3GD20_Y_L = 0x2A;
    const byte L3GD20_Y_H = 0x2B;
    const byte L3GD20_Z_L = 0x2C;
    const byte L3GD20_Z_H = 0x2D;
    const byte L3GD20_RW = 0x80;
    const byte L3GD20_MS = 0x40;
};

extern L3GD20 gyro;

#endif
