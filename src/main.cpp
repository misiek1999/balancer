#include <Arduino.h>
#include <SoftwareSerial.h>
#include "pinout.h"
#include "pid.h"
#include "connection.h"
// Main loop delay parameters
unsigned long start_loop_time;
#define DELTA_TIME 100
// PID parameters
#define KP 50
#define KI 120
#define KD 0//0.02f
Pid pid(KP, KI, KD);
//#define STABLE_ANGLE 23.8f//-5.0f
#define STABLE_ANGLE -2.5f//6.2f
//Motor angle stering
Connector connect;

#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "MadgwickAHRS.h"
Madgwick mad;
//#define DEBUG_PRINT
MPU6050 mpu;
// orientation/motion vars
Quaternion quat;     // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  int16_t aix, aiy, aiz;
  int16_t gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;

//int16_t mx, my, mz;
unsigned long microsPerReading, microsPrevious, microsNow;

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
int16_t motor_ster = 0;
void setup()
{
    Serial.begin(115200);
        Serial.println("Start init");


    //Motor pinout
    pinMode(MOTOR_1_LEFT, OUTPUT);
    pinMode(MOTOR_1_RIGHT, OUTPUT);
    pinMode(MOTOR_2_LEFT, OUTPUT);
    pinMode(MOTOR_2_RIGHT, OUTPUT);
    analogWrite(MOTOR_1_RIGHT, 0);
    analogWrite(MOTOR_2_RIGHT, 0);
    analogWrite(MOTOR_1_LEFT, 0);
    analogWrite(MOTOR_2_LEFT, 0);
    mad.begin(25);
    //Mpu init
    Wire.begin();
    Wire.setClock(400000);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    mpu.setFullScaleGyroRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleAccelRange(MPU6050_GYRO_FS_250);
    //mpu.setRate(25);
        // // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)

    connect.setup();
      microsPerReading = 1000000 / 25;
  microsPrevious = micros();
  Serial.println("Initlization finished!");
}

void loop()
{
    start_loop_time = millis();
    connect.run();
      microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
      microsPrevious = microsNow;
        mpu.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);
        // Serial.print(ax);Serial.print('\t');
        // Serial.print(ay);Serial.print('\t');
        // Serial.println(az);
        // quat.w = q0;
        // quat.x = q1;
        // quat.y = q2;
        // quat.z = q3;
        mad.updateIMU(gx, gy, gz, ax, ay, az);
        ypr[1]= mad.getPitch();
        // mpu.dmpGetGravity(&gravity, &quat);
        // mpu.dmpGetYawPitchRoll(ypr, &quat, &gravity);
#ifndef DEBUG_PRINT
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
#endif
        Serial.print(STABLE_ANGLE + (ypr[1]) + 0);
        Serial.print("\t");
        Serial.print(motor_angle);
        Serial.print("\t");
        motor_ster = pid.calculate(STABLE_ANGLE + (ypr[1]) + motor_angle);
        //Serial.println(motor_angle);
        Serial.println(motor_ster);
  }
    #define MIN_ROTATION 10
    if(motor_ster > 0&& motor_ster < MIN_ROTATION)
        motor_ster= MIN_ROTATION;
        else
    if(motor_ster < 0&& motor_ster >- MIN_ROTATION)
        motor_ster= -MIN_ROTATION;
    if (motor_ster > 0){
        analogWrite(MOTOR_1_RIGHT, motor_ster);
        analogWrite(MOTOR_2_RIGHT, motor_ster);
        digitalWrite(MOTOR_1_LEFT, LOW);
        digitalWrite(MOTOR_2_LEFT, LOW);
    }
    if (motor_ster < 0){
        //motor_ster = motor_ster / -1;
        // digitalWrite(MOTOR_1_RIGHT, LOW);
        // digitalWrite(MOTOR_2_RIGHT, LOW);
        // analogWrite(MOTOR_1_LEFT, motor_ster);
        // analogWrite(MOTOR_2_LEFT, motor_ster);
        analogWrite(MOTOR_1_RIGHT, 255 +motor_ster);
        analogWrite(MOTOR_2_RIGHT, 255 +motor_ster);
        digitalWrite(MOTOR_1_LEFT, HIGH);
        digitalWrite(MOTOR_2_LEFT, HIGH);
    }
    // put your main code here, to run repeatedly:
    //delay(DELTA_TIME - (millis() - start_loop_time));
}