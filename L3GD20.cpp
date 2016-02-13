#include <SPI.h>
#include "L3GD20.h"

L3GD20 gyro;

//////write your code///////
void L3GD20::L3GD20_write(byte reg, byte val)
{
  digitalWrite(L3GD20_CS, LOW);
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(L3GD20_CS, HIGH);

}

byte L3GD20::L3GD20_read(byte reg)
{
  byte ret = 0;

  digitalWrite(L3GD20_CS, LOW);
  SPI.transfer(reg | L3GD20_RW);
  ret = SPI.transfer(0);
  digitalWrite(L3GD20_CS, HIGH);
  return ret;
}

void L3GD20::GetRowValue(short *x, short *y, short *z)
{
  short X, Y, Z;

  X = L3GD20_read(L3GD20_X_H);
  *x = X = (X << 8) | L3GD20_read(L3GD20_X_L);
  Y = L3GD20_read(L3GD20_Y_H);
  *y = Y = (Y << 8) | L3GD20_read(L3GD20_Y_L);
  Z = L3GD20_read(L3GD20_Z_H);
  *z = Z = (Z << 8) | L3GD20_read(L3GD20_Z_L);
}

void L3GD20::GetPhysicalValue_deg(float *x, float *y, float *z)
{
  short x_raw, y_raw, z_raw;

  GetRowValue(&x_raw, &y_raw, &z_raw);

  *x = (float)(x_raw - offsetxGyro) * 0.00875; // +-250dps
  //x *= 0.0175;// +-500dps
  //x *= 07;  // +-2000dps
  *y = (float)(y_raw - offsetyGyro) * 0.00875; // +-250dps
  *z = (float)(z_raw - offsetzGyro) * 0.00875; // +-250dps

}

void L3GD20::Init(int CS1, DRBW settingDRBW)
{
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  SPI.begin();
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  L3GD20_CS = CS1;

  int i;
  short tmpx, tmpy, tmpz;
  float avrx, avry, avrz;

  avrx = avry = avrz = 0;

  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 8MHz/8 = 1MHz; (max 10MHz)


  while (!Serial) {
  }

  Serial.println(L3GD20_read(L3GD20_WHOAMI), HEX); // should show D4

  L3GD20_write(L3GD20_CTRL1, B00001111 | (settingDRBW << 4));
  //   |||||||+ X axis enable
  //   ||||||+- Y axis enable
  //   |||||+-- Z axis enable
  //   ||||+--- PD: 0: power down, 1: active
  //   ||++---- BW1-BW0: cut off 12.5[Hz]
  //   ++------ DR1-DR0: ODR 95[HZ]

  for (i = 0; i < OFFSET_NUM; i++)
  {
    GetRowValue(&tmpx, &tmpy, &tmpz);
    avrx += tmpx;
    avry += tmpy;
    avrz += tmpz;
  }

  offsetxGyro = avrx / OFFSET_NUM;
  offsetyGyro = avry / OFFSET_NUM;
  offsetzGyro = avrz / OFFSET_NUM;
}

void L3GD20::SetFrequencyOfHPF(byte frequency)
{
  L3GD20_write(L3GD20_CTRL5, B00010000);
  L3GD20_write(L3GD20_CTRL2, frequency);
}


