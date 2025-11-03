#ifndef ESCCONTROLLER_H
#define ESCCONTROLLER_H

#include <Arduino.h>

class ESCController {
public:
    ESCController(int pin, int channel, int freq = 400, int min_us = 1000, int max_us = 2000, int max_thr = 1000);
    
    void setLimit(int limit_min_thr, int limit_max_thr);
    void setThr(int target_thr);
    int getThr();
    void setThrRelative(int relative_thr);
    void reset(int target_thr = 0);

private:
    int pin;
    int channel;
    int freq;
    int min_us;
    int max_us;
    int max_thr;
    int limit_min_thr;
    int limit_max_thr;
    int target_thr;

    void setPWM(int us);
};

#endif