/***
   
    F I R M W A R E

    SDP2016 Group 1

***/

#define FW_DEBUG // Comment out to remove serial debug chatter

#include "CommandSet.h"
#include "State.h"
#include "Processes.h"
#include "SDPArduino.h"
#include <Wire.h>
#include <Arduino.h>
#include "addresses.h"

State state;
CommandSet command_set;
Processes processes;

/***
    LOOP & SETUP
***/
void setup()
{
    state.setup();
    command_set.setup();
    processes.setup();

    SDPsetup();

    Serial.begin(115200);
    Serial.println(F("STARTUP"));
}

void loop()
{ 
    command_set.readSerial();

    /* Run through schedule */
    processes.run_processes();
}
