/***
   
    F I R M W A R E

    SDP2016 Group 1

***/

#define FW_DEBUG // Comment out to remove serial debug chatter


#include <Wire.h>
#include <Arduino.h>

#include "CommandSet.h"
#include "State.h"
#include "Processes.h"
#include "SDPArduino.h"
#include "addresses.h"

State state;
CommandSet command_set;
Processes processes;

/*
  Blink status LED every second or so
*/
void heartbeat_f(size_t);
process heartbeat = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 999,
    .enabled    = true,
    .callback   = &heartbeat_f
};

/*
  Rapidly check motor rotations and speeds
*/
void poll_encoders_f(size_t);
process poll_encoders = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 50,
    .enabled    = true,
    .callback   = &poll_encoders_f
};

/*
  Instantaneously correct motor speeds
*/
void check_motors_f(size_t);
process check_motors = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 50,
    .enabled    = false,
    .callback   = &check_motors_f
};


/***
    SETUP
***/
void setup()
{
    Serial.begin(115200);

    state.setup();
    command_set.setup();
    processes.setup();
    SDPsetup();

    processes.add(&poll_encoders);
    processes.add(&check_motors);
    processes.add(&heartbeat);

    delay(500);
    Serial.println(F("STARTUP"));
}

/***
    LOOP
***/
void loop()
{
    /* async */
    command_set.readSerial();
    /* sync */
    processes.run();
}


/***
    PROCESS FUNCTIONS
***/

void heartbeat_f(size_t pid)
{
    // Toggles the LED
    command_set.led();
}

void poll_encoders_f(size_t pid)
{
    Wire.requestFrom(encoder_i2c_addr, motor_count);

    for (int i = 0; i < motor_count; i++) {
        byte delta = (byte) Wire.read();
        if (state.motors[i]->power < 0) {
            if (delta) {
                delta = 0xFF - delta;
            }
            state.motors[i]->disp -= delta;
        } else {
            state.motors[i]->disp += delta;
        }
        /*
          Update speed using the time now and the time we last checked
        */

        process* self = processes.get_by_id(pid);

        state.motors[i]->speed = (float) delta /
                                 (((float) millis() -
                                   (float) self->last_run) / 1000.0);

        if (state.motors[i]->power < 0) {
            state.motors[i]->speed *= -1;
        }
    }
}


void check_motors_f(size_t pid)
{
    // TODO
}


void check_rotation(size_t pid)
{
    long current_delta = 0;

    for (size_t i = 0; i < motor_count; i++) {
        current_delta += abs(state.motors[i]->disp - state.initial_displacement[i]);
    }

    current_delta /= motor_count;

    if (current_delta >= state.rotation_delta) {
        #ifdef FW_DEBUG
        Serial.println("Ending rotation");
        #endif
        motorAllStop();
        processes.disable(pid);
    }
}
