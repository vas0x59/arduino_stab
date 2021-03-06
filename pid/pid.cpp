#include "pid.h"

PID::PID()
{
}
float PID::_calc(float error)
{
    // float error =
    float v = 0;
    unsigned long now = millis();
    unsigned long d_time = (now - prev_time);
    integral += error * d_time;
    if (first_i == true)
    {
        first_i = false;
        v = error * kP;
    }
    else
    {
        float vP = kP * error;
        float vI = kI * integral;
        float vD = kD * ((error - prev_error) / d_time);
        v = vP + vI + vD;
        // v = kP*error + kI*() + kD*();
    }

    prev_error = error;
    prev_time = now;
    return v;
}
float PID::calc(float error)
{
    // float error =
    return _calc(error);
}
float PID::calc(float value, float set_point)
{
    float error = set_point - value;
    return _calc(error);
}