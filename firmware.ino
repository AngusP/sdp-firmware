/***
   
    F I R M W A R E

    SDP2016 Group 1

***/

#define FW_DEBUG                             // Comment out to remove serial debug chatter

#include "CommandSet.h"
#include "State.h"
#include "SDPArduino.h"
#include <Wire.h>
#include <Arduino.h>

#define sensorAddr 0x39  // Sensor physical address on the power board - 0x39
#define ch0        0x43  // Read ADC for channel 0 - 0x43
#define ch1        0x83  // Read ADC for channel 1 - 0x83

#define encoder_i2c_addr 5

/***
    STRUCTURES & PROTOTYPES
***/
void poll_encoders();
void null_process();

State state;
CommandSet command_set;

/* 
   Process structure, for defining a 
   clock synchronous process 
*/
struct process {
    unsigned long last_run, interval;
    void (*callback)();
};

/*
  Toggle LED ~1 time per second
  999 so it is unlikely to coincide
  with things that actually have to 
  happen once per second
*/
struct process heartbeat = {
    .last_run = 0,
    .interval = 999,
    .callback = &(command_set.led)
};

struct process update_speeds = {
    .last_run = 0,
    .interval = 50,
    .callback = &poll_encoders
};

struct process monitor_motors = {
    .last_run = 0,
    .interval = 100,
    .callback = &null_process
};

struct process* tasks[] = {
    &heartbeat,
    &update_speeds,
    &monitor_motors
};


/***
    LOOP & SETUP
***/
void setup()
{
    state.setup();
    command_set.setup();

    SDPsetup();

    Serial.begin(115200);
    Serial.println(F("STARTUP"));
}

void loop()
{ 
    command_set.readSerial();


    /* Run through schedule */
    for (int i=0; i<(sizeof(tasks)/sizeof(struct process*)); i++){
        if (millis() >= (tasks[i]->last_run + tasks[i]->interval)){
            tasks[i]->callback();
            tasks[i]->last_run = millis();
        }
    }


    /* Milestone 1 */

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

/*
  Callback for the update_speeds process
*/
void poll_encoders()
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
            (((float) millis() - (float) update_speeds.last_run) / 1000.0);

        if (state.motors[i]->power < 0) state.motors[i]->speed *= -1;
    }
}


void null_process()
{
    // Do nothing
}
