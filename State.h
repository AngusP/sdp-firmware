#ifndef FIRMWARE_STATE_H
#define FIRMWARE_STATE_H

#include <Arduino.h>
#include "Processes.h"

#define motor_count 3

/*
   Generic motor structure, carries around all needed information.

   IT IS ASSUMED that the encoders are connected in the same order
   that the motors are added to the motors array.
*/
typedef struct {
    int port;
    int power;
    int desired_power;
    int direction;
    long disp;
    int disp_delta;
    float speed;
    float delta_speed;
    unsigned long last_write;
} motor;

class State {

public:
    void setup();
    
    int battery;
    uint8_t status_led;
    const uint8_t status_led_pin = 13;
    float heading;
    
    motor* motors[motor_count];
    
    // Rotation stuff
    long initial_displacement[motor_count];
    long rotation_delta;
    process* rotation_process;

    // Allows kicking and grabbing to execute without blocking
    process* kick_grab_handler;
    int kg_handler_action;
    
    // For holonomics
    static const float velo_coupling_mat[3][3];
};

extern State state;

#endif //FIRMWARE_STATE_H
