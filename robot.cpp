/***
   
    R O B O T . C P P

***/

#include "robot.h"
#include "SDPArduino.h"
#include <math.h>

/*** 
     PROCESS DEFINITIONS
***/

/*
  Blink status LED every second or so
*/
void heartbeat_f(pid_t);
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
void update_motors_f(pid_t);
process update_motors = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 50,
    .enabled    = true,
    .callback   = &update_motors_f
};

/*
  Instantaneously correct motor speeds
*/
void check_motors_f(pid_t);
process check_motors = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 50,
    .enabled    = false,
    .callback   = &check_motors_f
};



/*** 
     PROCESS FUNCTIONS
***/

void heartbeat_f(pid_t pid)
{
    // Toggles the LED
    command_set.led();
}

void update_motors_f(pid_t pid)
{
    /*
      Poll encoders and update speed
    */
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
        
        float old_speed = state.motors[i]->speed;
        
        process* self = processes.get_by_id(pid);
        
        state.motors[i]->speed = (float) delta /
            (((float) (millis() - self->last_run)) / 1000.0);

        if (state.motors[i]->power < 0) {
            state.motors[i]->speed *= -1.0;
        }

        state.motors[i]->delta_speed =
            state.motors[i]->speed - old_speed;
    }
}


void check_motors_f(pid_t pid)
{
    /* Stall (or wall) detection */
    
    for (int i = 0; i < motor_count; i++) {
        if ((millis() - state.motors[i]->last_write) > state.stall_spool_time
            && state.motors[i]->delta_speed < state.stall_threshold
            && state.motors[i]->power != 0) {
            
            Serial.println();
            Serial.print(F("STALL motor "));
            Serial.print(i);
            Serial.print(F("! "));
            Serial.println(state.motors[i]->delta_speed);
            write_powers(0);
        }
    }
}


void check_rotation_f(pid_t pid)
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

/*** 
     REGISTER WITH PROCESS OBJECT
***/

void Robot::register_processes()
{
    processes.add(&update_motors);
    processes.add(&check_motors);
    processes.add(&heartbeat);
}


