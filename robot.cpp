/***
   
    R O B O T . C P P

***/

#include "robot.h"
#include "SDPArduino.h"
#include "State.h"
#include <math.h>

/*** 
     PROCESS DEFINITIONS
***/

/*
  Blink status LED every second or so
*/
const char heartbeat_l[] = "heartbeat";
void heartbeat_f(pid_t);
process heartbeat = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 999,
    .enabled    = true,
    .callback   = &heartbeat_f,
    .label      = heartbeat_l
};

/*
  Rapidly check motor rotations and speeds
*/
const char update_motors_l[] = "update motors";
void update_motors_f(pid_t);
process update_motors = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 50,
    .enabled    = true,
    .callback   = &update_motors_f,
    .label      = update_motors_l
};

/*
  Instantaneously correct motor speeds
*/
const char check_motors_l[] = "check motors";
void check_motors_f(pid_t);
process check_motors = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 150,
    .enabled    = false,
    .callback   = &check_motors_f,
    .label      = check_motors_l
};

/*
  Run through specified rotation
*/
const char exec_rotation_l[] = "exec rotate";
void exec_rotation_f(pid_t);
process exec_rotation = {
    .id         = 0,
    .last_run   = 0,
    .interval   = 60,
    .enabled    = false,
    .callback   = &exec_rotation_f,
    .label      = exec_rotation_l
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
        state.motors[i]->disp_delta += delta;

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
    
    /*for (int i = 0; i < motor_count; i++) {
        float spd_threshold = (state.stall_gradient * fabsf(state.motors[i]->power))
            + state.stall_constant;

        if (fabsf(state.motors[i]->speed) < spd_threshold &&
            abs(state.motors[i]->power) >= 76) {
            state.stall_count += 10;
        }
    }

    if (state.stall_count > 0) state.stall_count--;

    if (state.stall_count > 200) {
        Serial.println(F("STALL!"));
        write_powers(0);
        state.stall_count = 0;
    }*/

    motor*mtr;

    float c_norm = 0.0f,
          r_norm = 0.0f,
          d_norm = 0.0f;

    for (int i = 0; i < motor_count; i++) {
        mtr = state.motors[i];

        c_norm += pow(mtr->power, 2);
        r_norm += pow(mtr->disp_delta, 2);
        d_norm += pow(mtr->desired_power, 2);
    }

    r_norm = sqrt(r_norm);

    // TODO if r_norm is too low for the power, we have stalled

    c_norm = sqrt(c_norm);
    d_norm = sqrt(d_norm);

    for (int i = 0; i < motor_count; i++) {
        mtr = state.motors[i];

        //mtr->power = (int) (mtr->desired_power + (r_norm / c_norm) * r[i]);
        mtr->power = (int) (mtr->desired_power + d_norm * ((mtr->power / c_norm) - (mtr->disp_delta / r_norm)));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.println(mtr->power);

        mtr->disp_delta = 0;
    }
    write_powers();
}


void exec_rotation_f(pid_t pid)
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
    processes.add(&exec_rotation);

    state.rotation_process = &exec_rotation;
}



