#ifndef CONN
#define CONN
    #include <Arduino.h>
    #include <pinout.h>
    #include <NeoSWSerial.h>
    #include <SoftwareSerial.h>
    // File with function to ble communication
    //#include "bleConnection.h" // Not necessery
    // File with class and function to radio communication via nrf24
    #define STERING_MOTOR_ANGLE 1.0f
    #define STERING_MOTOR_ROTATION 100.0f
    enum class connection_types{
        BLE =1,
        RF24 =2,
        NO_CONNECTED =3
    };
    class Connector{
        connection_types type_of_connection;
        // Buffer for received data
        NeoSWSerial SerialBLE;
        char buff[32];
        void read_data_and_do_action_BLE();
        unsigned long last_time;
        void save_last_connection_time();
    public:
        Connector();
        bool setup();
        bool run();
        const connection_types get_connection_type()const;
        unsigned long last_received_data_time();
    };

    extern float motor_angle;
    extern float motor_rotation_force;
#endif