/***
   
    R O B O T . H

***/

#ifndef ROBOT_H
#define ROBOT_H

#include <Wire.h>
#include <Arduino.h>
#include "Processes.h"
#include "CommandSet.h"
#include "addresses.h"
#include "SDPArduino.h"

class Robot {

public:
    void register_processes();

};

extern Robot bot;

#endif // ROBOT_H

