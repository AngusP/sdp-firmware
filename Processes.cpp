
#define FW_DEBUG // Comment out to remove serial debug chatter

#include "Processes.h"
#include "CommandSet.h"
#include "State.h"
#include <Arduino.h>
#include <Wire.h>
#include "addresses.h"

// Declaring these structs here as an alternative to malloc

/*
 * Toggle LED ~1 time per second 999 so it is unlikely to coincide
 * with things that actually have to happen once per second
 */
static struct process heartbeat_process = {
    .last_run   = 0,
    .interval   = 999,
    .enabled    = true,
    .callback   = &Processes::heartbeat
};

static struct process poll_encoders_process = {
    .last_run   = 0,
    .interval   = 50,
    .enabled    = true,
    .callback   = &Processes::poll_encoders
};

static struct process check_motors_process = {
    .last_run   = 0,
    .interval   = 100,
    .enabled    = false,
    .callback   = &Processes::check_motors
};

static struct process milestone_1_process = {
    .last_run   = 0,
    .interval   = 50, // TODO is this a reasonable number?
    .enabled    = true,
    .callback   = &Processes::milestone_1
};


struct process* Processes::processes[PROCESS_COUNT];

void Processes::setup()
{
    processes[HEARTBEAT_PROCESS] = &heartbeat_process;
    processes[POLL_ENCODERS_PROCESS] = &poll_encoders_process;
    processes[CHECK_MOTORS_PROCESS] = &check_motors_process;
    processes[MILESTONE_1_PROCESS] = &milestone_1_process;
}

void Processes::run()
{
    for (size_t i = 0; i < PROCESS_COUNT; i++) {
        if (processes[i]->enabled && millis() >= (processes[i]->last_run + processes[i]->interval)){
            processes[i]->callback();
            processes[i]->last_run = millis();
        }
    }
}

/******************************************** Process management routines *********************************************/

void Processes::disable_process(size_t process_id)
{
    processes[process_id]->enabled = false;
}

void Processes::enable_process(size_t process_id)
{
    processes[process_id]->enabled = true;
}

void Processes::change_process(size_t process_id, void (*callback)(), unsigned long interval)
{
    struct process* process = processes[process_id];
    change_process(process_id, callback);
    change_process(process_id, interval);
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