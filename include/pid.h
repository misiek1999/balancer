#ifndef PID_H
#define PID_H
    #include <Arduino.h>
    #define OUTPUT_MAX_LIMIT 255
    #define OUTPUT_MIN_LIMIT -255
    #define MAX_ERROR_SUM 200
    #define MIN_ERROR_SUM -200
    class Pid{
        float kp;
        float ki;
        float kd;
        unsigned long last_time;
        float last_error;
        float error_sum;
    public:
        Pid(float, float, float);
        float calculate(float);

    };


#endif