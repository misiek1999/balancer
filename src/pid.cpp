#include "pid.h"

Pid::Pid(float _kp, float _ki, float _kd)
{
    //Setup parameters
    kp = _kp;
    ki = _ki;
    kd = _kd;
    last_time = millis();
    last_error = 0;
    error_sum = 0;
}

float Pid::calculate(float error)
{
    unsigned long now = millis();
    float delta_time = (now - last_time) * 0.001;
    error_sum += (error + last_error)/2 *ki * delta_time;
    if (error_sum > MAX_ERROR_SUM)
        error_sum = MAX_ERROR_SUM;
    if (error_sum < MIN_ERROR_SUM)
        error_sum = MIN_ERROR_SUM;
    float output = kp * error + error_sum + kd * (last_error - error) / delta_time;
    if (output > OUTPUT_MAX_LIMIT)
        output = OUTPUT_MAX_LIMIT;
    if (output < OUTPUT_MIN_LIMIT)
        output = OUTPUT_MIN_LIMIT;
    last_error = error;
    last_time = now;
    return output;
}