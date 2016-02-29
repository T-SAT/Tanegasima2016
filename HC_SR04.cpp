#include "HC_SR04.h"

HC_SR04 sonic;

void HC_SR04::Init(int trigpin, int echopin, int ctm) 
{
    pinMode(trigpin, OUTPUT);
    pinMode(echopin, OUTPUT);
    
    TRIGPIN = trigpin;
    ECHOPIN = echopin; 
    CTM = ctm;
}

float HC_SR04::ReadLength(void) 
{
    int dur;
    float dis;

    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(CTM);
    digitalWrite(TRIGPIN, LOW);

    dur = pulseIn(ECHOPIN, HIGH);
    dis = (float)dur * 0.017;
    dis *= 0.01;

    return(dis);
}
