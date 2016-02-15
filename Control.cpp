#include "Control.h"

Control control;

float Control::getDt(void)
{
  static long lastTime = 0;

  long nowTime = millis();
  float time = (float)(nowTime - lastTime);
  time = max(time, 20);  //timeは20[us]以上
  time /= 1000;  //[usec] => [sec]
  lastTime = nowTime;

  return ( time );
}

void Control::SetPIDGain(float pGain, float iGain, float dGain)
{
  PGain = pGain;
  IGain = iGain;
  DGain = dGain;
}

float Control::PIDControl(float dCommand, float dVal)
{
  static float s_dErrIntg = 0;
  static float s_iErrDet = 0;
  static float dRet = 0;
  float dErr, iErr;
  float dt;

  Serial.print(PGain);
  Serial.print("\t");
  Serial.print(IGain);
  Serial.print("\t");
  Serial.println(DGain);
  dt = getDt();
  // 誤差
  dErr = dCommand - dVal;
  // 誤差積分
  s_dErrIntg += dErr * dt;
  // 誤差微分
  iErr = dErr - s_iErrDet;
  s_iErrDet = dErr;

  // 制御入力
  dRet = PGain * dErr + IGain * s_dErrIntg + DGain * iErr;

  return (dRet);
}

