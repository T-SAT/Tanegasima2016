#include "XBEE.h"

XBEE::XBEE(uint8_t receivePin, uint8_t transmitPin) : SoftwareSerial(receivePin, transmitPin)
{
    SoftwareSerial(receivePin, transmitPin);
}

void XBEE::TransferData(float data[], int num)
{
    int i;

    for(i = 0; i < num; i++) {
        print(data[i]);
        print(",");
    }

    print(millis());
    println();
}

