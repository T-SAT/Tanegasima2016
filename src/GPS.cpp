#include "GPS.h"

GPS::GPS(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic)
  : SoftwareSerial(receivePin, transmitPin, inverse_logic)
{
  SoftwareSerial(receivePin, transmitPin);
  begin(4800);
  while(available())
    read();
}

GPS::GPS(uint8_t receivePin, uint8_t transmitPin, int baudrate)
  : SoftwareSerial(receivePin, transmitPin)
{
  SoftwareSerial(receivePin, transmitPin);
  begin( baudrate);
  while(available())
    read();
}

IS_GPSDATA_AVAILABLE GPS::ReceiveGPSData(void)
{
  static unsigned int ReceiveGPSNum = 0;
  static unsigned long lastTime = millis();
  unsigned long nowTime = millis();

  ReceiveGPSNum += available();
  while (available())
    encode(read());
  if (ReceiveGPSNum >= 63) {
    while (available())
      read();
    ReceiveGPSNum = 0;
    if ((nowTime - lastTime) >= SamplingRate) {
      lastTime = millis();
      return (GPSDATA_AVAILABLE);
    }
  }
  return (GPSDATA_NOTAVAILABLE);
}

void GPS::gelay(unsigned long ms)
{
  unsigned long start = millis();
  do {
    while (available()) 
      encode(read());
  } while (millis() - start < ms);
}


