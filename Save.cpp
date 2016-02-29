#include <SD.h>
#include <SPI.h>
#include "Save.h"

Save save;

int Save::InitSDSlot(int sd_cspin)
{
    pinMode(10, OUTPUT);
    Serial.print("Initializing SD card..."); 
    if(!SD.begin(sd_cspin)) {
        Serial.println("initialization failed!");
        return(-1);
    }
    Serial.println("initialization done.");
    return(0);
}

int Save::OnSD(char *filename, float *data, int num)
{
    File datafile;
    int i;

    datafile = SD.open(filename, FILE_WRITE);
    if(!datafile) {
        char str[100];

        sprintf(str, "%s can't open.", filename);
        Serial.println(str);
        return(-1);
    } else {
        for(i = 0; i < num; i++) {
            datafile.print(data[i]);
            datafile.print(",");    
        }
        datafile.print(millis());
        datafile.println();
        datafile.close();
        return(0);
    }

}

int Save::OnSDStr(char *filename, char *str) 
{
    File datafile;
    int i;

    datafile = SD.open(filename, FILE_WRITE);
    if(!datafile) {
        char str[100];

        sprintf(str, "%s can't open.", filename);
        Serial.println(str);
        return(-1);
    }
    datafile.println(str);
    datafile.close();
    return(0);
}


