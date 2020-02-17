#include <Arduino.h>
#include <SoftwareSerial.h>
#include "pinout.h"
#include "pid.h"
// Main loop delay parameters
unsigned long start_loop_time;
#define DELTA_TIME 100
// PID parameters
#define KP 35
#define KI 130
#define KD 0.025
Pid pid(KP, KI, KD);
#define STABLE_ANGLE 0.5f

#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#define DEBUG_PRINT
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion quat;     // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}
int16_t motor_ster = 0;
void setup()
{
    Serial.begin(115200);
    //Motor pinout
    pinMode(MOTOR_1_LEFT, OUTPUT);
    pinMode(MOTOR_1_RIGHT, OUTPUT);
    pinMode(MOTOR_2_LEFT, OUTPUT);
    pinMode(MOTOR_2_RIGHT, OUTPUT);
    analogWrite(MOTOR_1_RIGHT, 0);
    analogWrite(MOTOR_2_RIGHT, 0);
    analogWrite(MOTOR_1_LEFT, 0);
    analogWrite(MOTOR_2_LEFT, 0);
    //Mpu init
    Wire.begin();
    Wire.setClock(400000);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(MPU_interupt, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // // Calibration Time: generate offsets and calibrate our MPU6050
        // mpu.CalibrateAccel(6);
        // mpu.CalibrateGyro(6);
        // mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(MPU_interupt));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(MPU_interupt), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop()
{
    start_loop_time = millis();
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

        mpu.dmpGetQuaternion(&quat, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quat);
        mpu.dmpGetYawPitchRoll(ypr, &quat, &gravity);
#ifndef DEBUG_PRINT
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
#endif
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        motor_ster = pid.calculate(STABLE_ANGLE - (ypr[1] * 180 / M_PI));
        Serial.println(motor_ster);
    }
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