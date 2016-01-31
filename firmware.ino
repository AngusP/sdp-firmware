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
#include "robot.h"

State state;
CommandSet command_set;
Processes processes;
Robot bot;


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

    bot.register_processes();

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
