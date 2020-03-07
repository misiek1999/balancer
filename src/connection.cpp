#include "connection.h"
float motor_angle;
float motor_rotation_force;
Connector::Connector() :
SerialBLE(SSreceiver, SStransmitt)
{
    type_of_connection = connection_types::NO_CONNECTED;
    motor_angle = 0;
    SerialBLE.begin(9600);
}

bool Connector::setup()
{
    type_of_connection = connection_types::BLE;
    return true;
}

//TODO: add receive data from radio
bool Connector::run()
{

    uint8_t ind_buff = 0;
    buff[0] = '\0';
    if (type_of_connection == connection_types::BLE)
    {
        //Serial.println("BLE");
        if (SerialBLE.available())
        {
            ind_buff = SerialBLE.readBytesUntil('*', buff, sizeof(buff));
            buff[ind_buff] = '\0';
            Serial.println(buff);
        }
        else
        {
            return false;
        }
        if (buff[0] == '\0')
            return true;
        else
            read_data_and_do_action_BLE();
        return true;

    }

    // No receive any data from master
    return false;
}

const connection_types Connector::get_connection_type() const
{
    return type_of_connection;
}
#define MAX_BUFF_ITR 10
void Connector::read_data_and_do_action_BLE()
{
    switch (buff[0])
    {
        // FIRST PAD
    case 'P':
        if (buff[1] == 'X')
        {
            uint8_t buff_itr = 2;
            int16_t pad_x = 0;
            bool negative_number = false;
            while (buff[buff_itr] != 'Y')
            {
                if (buff[buff_itr] == '-')
                    negative_number = true;
                else
                {
                    pad_x = pad_x * 10;
                    pad_x += uint8_t(buff[buff_itr]) - 48;
                }
                buff_itr += 1;
                if (buff_itr > MAX_BUFF_ITR)
                    continue;
            }
            if (negative_number)
                pad_x *= -1;
            int16_t pad_y = 0;
            buff_itr += 1;
            negative_number = false;
            while (buff[buff_itr] != '\0')
            {
                if (buff[buff_itr] == '-')
                    negative_number = true;
                else
                {
                    pad_y = pad_y * 10;
                    pad_y += uint8_t(buff[buff_itr]) - 48;
                }
                buff_itr += 1;
                if (buff_itr > MAX_BUFF_ITR)
                    continue;
            }
            if (negative_number)
                pad_y *= -1;
            if (buff_itr > MAX_BUFF_ITR)
                return;

            Serial.println(pad_x);
            Serial.println(pad_y); // this one
            // Change speed of main motor
            motor_angle = map(pad_y, -127, 127, -STERING_MOTOR_ANGLE, STERING_MOTOR_ANGLE);
        }
        break;
    // Secornd PAD
    case 'R':
        if (buff[1] == 'X')
        {
            uint8_t buff_itr = 2;
            int16_t pad_x = 0;
            bool negative_number = false;
            while (buff[buff_itr] != 'Y')
            {
                if (buff[buff_itr] == '-')
                    negative_number = true;
                else
                {
                    pad_x = pad_x * 10;
                    pad_x += uint8_t(buff[buff_itr]) - 48;
                }
                buff_itr += 1;
                if (buff_itr > MAX_BUFF_ITR)
                    continue;
            }
            if (negative_number)
                pad_x *= -1;
            int16_t pad_y = 0;
            buff_itr += 1;
            negative_number = false;
            while (buff[buff_itr] != '\0')
            {
                if (buff[buff_itr] == '-')
                    negative_number = true;
                else
                {
                    pad_y = pad_y * 10;
                    pad_y += uint8_t(buff[buff_itr]) - 48;
                }
                buff_itr += 1;
                if (buff_itr > MAX_BUFF_ITR)
                    continue;
            }
            if (negative_number)
                pad_y *= -1;
            if (buff_itr > MAX_BUFF_ITR)
                return;
            Serial.println(pad_x); // this one
            Serial.println(pad_y);
            // Change speed of main motor
            motor_rotation_force = map(pad_x, 0, 180, -STERING_MOTOR_ROTATION, STERING_MOTOR_ROTATION);
        }

        break;
    case 'A':
        //Zmienia system na automatyczna zmianie biegów

        break;
    case 'B':
        //Manualne biegi, wybiera aktualny bieg

        break;
    case '1':
        //1
        break;
    case '2':
        //2
        break;
    case '3':
        //3
        break;
    case '4':
        //4
        break;
    case 'C':
        //Connected information
        break;
    default:
        Serial.println("E - Data not definied");
        break;
    }
}



void Connector::save_last_connection_time(){
    last_time = millis();
}


unsigned long Connector::last_received_data_time(){
    return last_time;
}