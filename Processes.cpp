

#include "Processes.h"
#include "CommandSet.h"
#include "State.h"
#include <Arduino.h>
#include <Wire.h>
#include "addresses.h"

struct process* Processes::processes[PROCESS_COUNT];

void Processes::setup()
{
    /*
     * Toggle LED ~1 time per second 999 so it is unlikely to coincide
     * with things that actually have to happen once per second
     */
    *(processes[HEARTBEAT_PROCESS]) = (struct process) {
        .last_run   = 0,
        .interval   = 999,
        .is_active  = true,
        .callback   = &Processes::heartbeat
    };

    *(processes[POLL_ENCODERS_PROCESS]) = (struct process) {
        .last_run   = 0,
        .interval   = 50,
        .is_active  = true,
        .callback   = &Processes::poll_encoders
    };

    *(processes[CHECK_MOTORS_PROCESS]) = (struct process) {
        .last_run   = 0,
        .interval   = 100,
        .is_active  = false,
        .callback   = &Processes::check_motors
    };

    *(processes[MILESTONE_1_PROCESS]) = (struct process) {
        .last_run   = 0,
        .interval   = 2, // TODO is this a reasonable number?
        .is_active  = true,
        .callback   = &Processes::milestone_1
    };
}

void Processes::run_processes()
{
    for (size_t i = 0; i < PROCESS_COUNT; i++) {
        if (processes[i]->is_active && millis() >= (processes[i]->last_run + processes[i]->interval)){
            processes[i]->callback();
            processes[i]->last_run = millis();
        }
    }
}

/******************************************** Process management routines *********************************************/

void Processes::deactivate_process(size_t process_id)
{
    processes[process_id]->is_active = false;
}

void Processes::activate_process(size_t process_id)
{
    processes[process_id]->is_active = true;
}

void Processes::change_process(size_t process_id, void (*callback)(), unsigned long interval)
{
    struct process* process = processes[process_id];
    process->callback = callback;
    process->interval = interval;
}

void Processes::change_process(size_t process_id, void (*callback)())
{
    processes[process_id]->callback = callback;
}

void Processes::change_process(size_t process_id, unsigned long interval)
{
    processes[process_id]->interval = interval;
}


/********************************************** Actual process handlers ***********************************************/

void Processes::heartbeat()
{
    // Toggles the LED
    command_set.led();
}

void Processes::poll_encoders()
{
    Wire.requestFrom(encoder_i2c_addr, motor_count);

    for (int i = 0; i < motor_count; i++) {
        byte delta = (byte) Wire.read();
        if (state.motors[i]->power < 0) {
            delta = 0xFF - delta;
            state.motors[i]->disp -= delta;
        } else {
            state.motors[i]->disp += delta;
        }
        /*
          Update speed using the time now and the time we last checked
        */

        state.motors[i]->speed = (float) delta /
                                 (((float) millis() - (float) processes[POLL_ENCODERS_PROCESS]->last_run) / 1000.0);

        if (state.motors[i]->power < 0) {
            state.motors[i]->speed *= -1;
        }
    }
}

void Processes::check_motors()
{

}

void Processes::milestone_1()
{
    if (state.receiving) {
        state.sending_index = 0;

        #ifdef FW_DEBUG
        Serial.println(F("Waiting for bytes..."));
        #endif

        /* Block and read serial */
        while (state.sending_index < state.num_bytes){
            if(Serial.available()){
                state.receive_bytes[state.sending_index++] = (byte) Serial.read();

                #ifdef FW_DEBUG
                Serial.print(F("Read 0x"));
                Serial.println(state.receive_bytes[state.sending_index-1], HEX);
                #endif
            }
        }
        state.sending = true;
        state.receiving = false;
        state.sending_index = 0;
        state.last_send = millis();
        state.time_period = (1.0/(long) state.send_frequency) * 1000.0;

        #ifdef FW_DEBUG
        Serial.println(F("Got it, thanks bub"));
        Serial.print(F("Writing at "));
        Serial.print(state.send_frequency);
        Serial.print(F(" ("));
        Serial.print(state.time_period);
        Serial.println(F(" s^-1)"));
        #endif
    }

    // Ewww.
    if (state.sending && (millis() >= (state.last_send + state.time_period))){
        if (state.sending_index < state.num_bytes) {
            // 0x45 is the address according to the milestone page
            Wire.beginTransmission(0x45);
            Wire.write(state.receive_bytes[state.sending_index++]);
            Wire.endTransmission();
        } else {
            state.sending = false;

            #ifdef FW_DEBUG
            Serial.print(F("Sent "));
            Serial.print(state.num_bytes);
            Serial.println(F(" bytes over the i2c bus"));
            #endif
        }
        state.last_send = millis();
    }
}