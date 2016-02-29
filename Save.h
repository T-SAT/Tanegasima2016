#ifndef SAVE_H_INCLUDED
#define SAVE_H_INCLUDED

class Save {
    public:
        int InitSDSlot(int sd_cspin);
        int OnSD(char *filename, float *data, int num); 
        int OnSDStr(char *filename, char *str);
}; 
extern Save save;
#endif
