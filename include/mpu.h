#ifndef MPU
#define MPU 
#include <Arduino.h>
#include <math.h>
    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    extern float deltat, sum;       // integration interval for both filter schemes
    extern uint32_t lastUpdate , firstUpdate ; // used to calculate integration interval
    extern uint32_t Now ;       // used to calculate integration interval

    //extern float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
    extern float q[4];    // vector to hold quaternion
    extern float eInt[3];     // vector to hold integral error for Mahony method



#endif
