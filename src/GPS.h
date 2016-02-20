#ifndef GPS_H_INCLUDED
#define GPS_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

typedef enum {
  GPSDATA_AVAILABLE = 0,
  GPSDATA_NOTAVAILABLE,
} IS_GPSDATA_AVAILABLE;

class GPS : public TinyGPSPlus, public SoftwareSerial {
  public :
    GPS(uint8_t receivePin, uint8_t trasmitPin, bool inverse_logic = false);
    GPS(uint8_t receivePin, uint8_t transmitPin, int baudrate);
    void SetSamplingRate(unsigned long samplingrate);
    IS_GPSDATA_AVAILABLE ReceiveGPSData(void);
    void gelay(unsigned long ms);
    
  private :
    int RXPin, TXPin;
    unsigned long SamplingRate = 1000;
};

#endif
