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
    .interval   = 50,
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
    /* Gradient Descent Motor Error correction */

    const double learning_rate = 0.1;
    
    motor* mtr;
    int i, largest_power = 0;
    double c_norm = 0.0,
           r_norm = 0.0,
           d_norm = 0.0,
           d_r_norm = 0.0;

    for (i=0; i < motor_count; i++) {
        mtr = state.motors[i];

        /* Calculate norms for our three vectors */
        c_norm += pow((double) mtr->power, 2.0);
        r_norm += pow((double) mtr->disp_delta, 2.0);
        d_norm += pow((double) mtr->desired_power, 2.0);

        if (abs(largest_power) < abs(mtr->desired_power))
            largest_power = mtr->desired_power;
    }

    if (d_norm == 0)
        return;

    c_norm = sqrt(c_norm);
    r_norm = sqrt(r_norm);
    d_norm = sqrt(d_norm);

    double c_unit[motor_count],
           r_unit[motor_count],
           d_unit[motor_count];

    double err_rd[motor_count];

    for (i=0; i < motor_count; i++) {
        mtr = state.motors[i];

        /* Calculate unit vectors */
        c_unit[i] = ((double) mtr->power)         / c_norm;
        r_unit[i] = ((double) mtr->disp_delta)    / r_norm;
        d_unit[i] = ((double) mtr->desired_power) / d_norm;

        /* r_i - d_i */
        err_rd[i] = r_unit[i] - d_unit[i];

        /* Norm of d-r */
        d_r_norm += pow((d_unit[i] - r_unit[i]), 2.0);
    }

    d_r_norm = sqrt(d_r_norm);
    double new_powers[motor_count];
    double np_largest = 0.0;

    for (i=0; i < motor_count; i++) {
        mtr = state.motors[i];

        /* 
           re-use the err variable to get the derivative of the 
           error function w.r.t the ith dimension.
        */
        err_rd[i] /= d_r_norm;
        
        new_powers[i] = c_unit[i] - (learning_rate * err_rd[i]);
        np_largest = fabs(np_largest) < fabs(new_powers[i]) ? new_powers[i] : np_largest;
    }

    for (i=0; i < motor_count; i++) {
        mtr = state.motors[i];
        mtr->power = largest_power * (int) round(new_powers[i] / np_largest);
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



