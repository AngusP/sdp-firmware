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
    .interval   = 500,
    .enabled    = true,
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

    const double learn_rate = 0.05;

    motor* mtr;
    int i;
    int largest_power = 0;

    float d_norm = 0.0; // Norm of desired power
    float c_norm = 0.0; // Norm of applied power
    float r_norm = 0.0; // Norm of perceived speed

    /* Calculate norms of our three vectors */
    for (i=0; i < motor_count; i++) {
        mtr = state.motors[i];
        
        d_norm += sq((float) mtr->desired_power);
        c_norm += sq((float) mtr->power);
        r_norm += sq((float) mtr->speed);

        if (largest_power < abs(mtr->desired_power))
            largest_power = abs(mtr->desired_power);
    }

    if (d_norm == 0)
        return;

    d_norm = sqrt(d_norm);
    c_norm = sqrt(c_norm);
    r_norm = sqrt(r_norm);

    float d_unit[motor_count],
          c_unit[motor_count],
          r_unit[motor_count];

    float err_dr[motor_count];
    float r_d_norm = 0.0; // Norm of r - d

    /* Generate unit vectors and begin building error vector */
    for (i=0; i < motor_count; i++) {
        mtr = state.motors[i];

        /* Calculate unit vectors */
        d_unit[i] = ((float) mtr->desired_power) / d_norm;
        c_unit[i] = ((float) mtr->power)         / c_norm;
        r_unit[i] = ((float) mtr->speed)         / r_norm;

        /* r_i - d_i */
        err_dr[i] = d_unit[i] - r_unit[i];

        /* If we're getting NaNs we can't do anything */
        if (isnan(err_dr[i])) return;

        /* Norm of d-r */
        r_d_norm += sq(r_unit[i] - d_unit[i]);

        /*
        Serial.print(d_unit[i]);
        Serial.print(" ");
        Serial.print(c_unit[i]);
        Serial.print(" ");
        Serial.print(r_unit[i]);
        Serial.print(" | ");
        Serial.print(err_dr[i]);
        Serial.println(" ");
        */
    }
    
    r_d_norm = sqrt(r_d_norm);

    // Serial.println(r_d_norm);

    float new_powers[motor_count];
    float largest_new = 0.0;

    for (i=0; i < motor_count; i++) {
        mtr = state.motors[i];

        if(r_d_norm != 0)
            err_dr[i] /= r_d_norm;

        /*
        Serial.print(err_dr[i]);
        Serial.print(" ");
        */

        new_powers[i] = c_unit[i] + (learn_rate * err_dr[i]);
        
        if (largest_new < fabs(new_powers[i]))
            largest_new = fabs(new_powers[i]);
    }

    // Serial.println("\n--");

    for (i=0; i < motor_count; i++) {
        mtr = state.motors[i];
        mtr->power = round((float) largest_power * new_powers[i]);

        Serial.print(mtr->power);
        Serial.print(" ");
        Serial.println(new_powers[i]);
    }

    Serial.println("--");

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



