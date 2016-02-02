#ifndef FIRMWARE_STATE_H
#define FIRMWARE_STATE_H

#include <Arduino.h>

#define motor_count 3

/*
   Generic motor structure, carries around all needed information.

   IT IS ASSUMED that the encoders are connected in the same order
   that the motors are added to the motors array.
*/
typedef struct {
    int port, power, direction;
    long disp;
    float speed;
} motor;

class State {

public:
    void setup();
    
    int battery;
    uint8_t status_led, status_led_pin;
    float heading;
    
    motor* motors[motor_count];
    
    // Rotation stuff
    long initial_displacement[motor_count];
    long rotation_delta;
};

extern State state;

#endif //FIRMWARE_STATE_H
