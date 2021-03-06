#ifndef PID_h
#define PID_h
#include <Arduino.h>

class PID
{
public:
    PID();
    float calc(float value, float set_point);
    float calc(float error);
    void init();
    float kP;
    float kI;
    float kD;

private:
    bool first_i = true;
    float prev_error = 0;
    unsigned long prev_time = 0;
    float _calc(float error);
    float integral = 0;
};
#endif