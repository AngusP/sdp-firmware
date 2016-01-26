#ifndef FIRMWARE_STATE_H
#define FIRMWARE_STATE_H

#include <Arduino.h>

#define motor_count 3

/*
   Generic motor structure, carries around all needed information.

   IT IS ASSUMED that the encoders are connected in the same order
   that the motors are added to the motors array below.
*/
struct motor {
    int port, power, direction;
    long disp;
    float speed;
};

class State
{
    public:
        void setup();

        int battery, status_led;
        uint8_t status_led_pin;
        float heading;

        struct motor* motors[motor_count];

        /* milestone hoops: */
        byte receive_bytes[250];
        int send_frequency, num_bytes, sending_index;
        bool receiving;
        bool sending;
        unsigned long last_send;
        long time_period;
};

extern State state;

#endif //FIRMWARE_STATE_H
