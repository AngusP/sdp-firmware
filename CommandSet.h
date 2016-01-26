#ifndef FIRMWARE_COMMANDSET_H
#define FIRMWARE_COMMANDSET_H

#include "State.h"
#include "SerialCommand.h"
#include "SDPArduino.h"
#include "Adafruit_NeoPixel.h"

class CommandSet
{
    public:
        void setup();

        static void readSerial();

        static void led();
        static void ping();
        static void help();
        static void move();
        static void stop();
        static void go();
        static void speeds();
        static void receive();
        static void pixels();
        static void kick();
        static void unrecognized(const char* command);
};

extern CommandSet command_set;


#endif //FIRMWARE_COMMANDSET_H
