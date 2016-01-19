/***
   
    F I R M W A R E

    SDP2016 Group 1

***/

#include "SerialCommand.h"
#include "SDPArduino.h"
#include <Wire.h>
#include <Arduino.h>

#define FW_DEBUG                             // Comment out to remove serial debug chatter

#define arduinoLED 13                        // Arduino LED on board
#define leftm 5                              // Left wheel motor
#define rightm 4                             // Right wheel motor
#define catcher 2                            // Catcher motor
#define kicker 3                             // Kicker motor
#define minpower 30                          // Minimum Speed of the motors in power%
#define default_power 50                     // If no power argument is given, this is the default

#define motor_direction -1                   // Motors are backwards. Go figure.

#define sensorAddr 0x39  // Sensor physical address on the power board - 0x39
#define ch0        0x43  // Read ADC for channel 0 - 0x43
#define ch1        0x83  // Read ADC for channel 1 - 0x83

#define encoder_i2c_addr 5

SerialCommand sCmd;                         // The SerialCommand object

//Global powers for both movement motors
int left_power = 0;                         // Speed of left wheel
int right_power = 0;                        // Speed of right wheel
int stop_flag = 0;

//These global variables are used to keep track of the short rotate function
int srot_power = default_power;             // Power on motor in short rotate
int interval = 300;                         // Defines the interval between commands in the short rotate functions
unsigned long function_run_time;            // Used in the Short Rotate Functions to measure time they have been running
int function_running = 0;                   // Used to determine if there is a function running and which function it is

//For the sensor function
unsigned long time_since_last_run = 0;
int sensor_state = 0;
boolean do_we_have_ball;
int initial_light_value = 0;
boolean initial_has_been_set = false;

//Kicking
int kicking = 0;
long kick_interval;
int kick_state = 0;

/***
    STRUCTURES & PROTOTYPES
***/
void init_commandset();

struct motor {
    int port, encoder, power, direction;
    float correction;
};

struct motor holo1;
struct motor holo2;
struct motor holo3;

/* 
   define an array of our drive motors,
   this generalises the motor control logic

   NB this is *not* itself a struct
*/
const int num_drive_motors = 3;
struct motor* driveset[num_drive_motors] = {
    &holo1,
    &holo2,
    &holo3
};

// Rotary encoders
int positions[num_drive_motors] = {0};


/***
    LOOP & SETUP
***/
void setup()
{
    pinMode(arduinoLED, OUTPUT);
    digitalWrite(arduinoLED, HIGH);

    SDPsetup();

    Serial.begin(115200);
    Serial.println("ACK");

    init_commandset();

    /* set up main drive motors */
    holo1.port = 5;
    holo1.encoder = 0;
    holo1.direction = 1;
    holo1.power = 0;
    holo1.correction = 0.0;

    holo2.port = 4;
    holo2.encoder = 1;
    holo2.direction = 1;
    holo2.power = 0;
    holo2.correction = 0.0;

    holo3.port = 3;
    holo3.encoder = 2;
    holo3.direction = 1;
    holo3.power = 0;
    holo3.correction = 0.0;
}

void loop() 
{ 
    /* 
       The program will enter the loop and first use the switch statement to check which
       function was set to be running last.
    */

    switch(function_running) {
        //Non script function
    case 0:
        break;

        //Left/Right short rotate and brake function are stopped after interval
    case 1:
        if (millis() > function_run_time + interval) {
            leftStop();
            rightStop();
            function_running = 0;
        }
        break;
    case 2:
        if (millis() > function_run_time + interval) {
            leftStop();
            rightStop();
            function_running = 0;
        }
        break;

    default :
        Serial.println("I shouldn't be here");
    }

    /* poll serial buffer & check against command set */
    sCmd.readSerial();

    if(kicking) {
        if(millis() > kick_interval) {
            switch(kick_state) {
            case(0):
                move_kick();
                kick_state = 1;
                kick_interval = kick_interval + 250;
                break;
            case(1):
                move_kick();
                kick_state = 2;
                kick_interval = kick_interval + 300;
                break;
            case(2):
                move_kick();
                kick_state = 3;
                kick_interval = kick_interval + 400;
                break;
            case(3):
                move_kick();
                kick_state = 0;
                break;
            }
        }
    }
  
    //Makes the sensor code run once per second. Change the int for different values
    if(millis() > time_since_last_run + 15) {
        switch(sensor_state) {
        case(0):
            readI2C(sensorAddr, ch0);
            sensor_state = 1;
            time_since_last_run = time_since_last_run + 15 ;
            break;
        case(1):
            readI2C(sensorAddr, ch0);
            sensor_state = 2;
            time_since_last_run = time_since_last_run + 15;
            break;
        case(2):
            //Serial.print("Channel 0 : ");
            readI2C(sensorAddr, ch0);
            sensor_state = 0;
            time_since_last_run = millis();
            break;
        }
    }
}



/***
    FUNCTIONS
***/
void init_commandset()
{
    /* etup callbacks for SerialCommand commands */
    sCmd.addCommand("ON",    led_on);          // Turns LED on
    sCmd.addCommand("OFF",   led_off);         // Turns LED off

    /* Movement commands */
    sCmd.addCommand("MOVE", run_engine);       // Runs wheel motors
    sCmd.addCommand("FSTOP", force_stop);      // Force stops all motors
    sCmd.addCommand("KICK", kick);        // Runs kick script
    sCmd.addCommand("CATCHUP", move_catchup);      // Runs catch script
    sCmd.addCommand("CATCHDOWN", move_catchdown);      // Runs catch script

    sCmd.addCommand("HAVEBALL", have_ball);    //Checks if we have the ball
    sCmd.addCommand("RESETHB", reset_have_ball); //Resets the intial value of the inital light sensor value
  
    sCmd.addCommand("SROTL", move_shortrotL);
    sCmd.addCommand("SROTR", move_shortrotR);

    /* Remote Control Commands */
    sCmd.addCommand("RCFORWARD", rc_forward);
    sCmd.addCommand("RCBACKWARD", rc_backward);
    sCmd.addCommand("RCROTATL", rc_rotateL);
    sCmd.addCommand("RCROTATR", rc_rotateR);

    /* Read from rotary encoders */
    sCmd.addCommand("GETE", getenc);

    sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched
}

void updateMotorPositions()
{
    /* 
       Request motor position deltas from encoder board
    */

    Wire.requestFrom(encoder_i2c_addr, num_drive_motors);
    for (int i = 0; i < num_drive_motors; i++) {
        positions[i] = (int8_t) Wire.read();
    }
}

void printMotorPositions()
{
    Serial.print("Encoders: ");
    for (int i = 0; i < num_drive_motors; i++) {
        Serial.print(positions[i]);
        Serial.print(' ');
    }
    Serial.println();
}

void getenc()
{
    updateMotorPositions();
    printMotorPositions();
}

//Read data from light sensor method
void readI2C(int portAddress, int channelAddr)
{
    switch(sensor_state) {
    case (0):
        Wire.beginTransmission(portAddress); 
        Wire.write(byte(0x18)); // Write command to assert extended range mode - 0x1D
        //delay(40);              // Write command to reset or return to standard range mode - 0x18
        break;
    case(1):
        Wire.write(byte(0x03)); // Power-up state/Read command register
        Wire.endTransmission(); // stop transmitting 
        //delay(70);
        break;
    case(2):
        Wire.beginTransmission(portAddress);             
        Wire.write(byte(channelAddr)); // Read ADC channel 0 - 0x43 || Read ADC channel 1 - 0x83
        Wire.endTransmission();        // stop transmitting
      
        Wire.requestFrom(portAddress, 1);
        int byte_in;
      
        while(Wire.available())    // slave may send less than requested
        { 
            byte_in = Wire.read();    // receive a byte as character
            Serial.println(byte_in);         // print the character
        
            if(initial_has_been_set != true) {
                initial_light_value = byte_in;
                initial_has_been_set = true;
            }
        
            do_we_have_ball = (byte_in - initial_light_value) > 30;
        }
        break;
    }
  
}

void have_ball() 
{
    Serial.println(do_we_have_ball);
}

void reset_have_ball()
{
    initial_has_been_set = false; 
}

// Test Commands
void led_on() 
{
    Serial.println("LED on");
    digitalWrite(arduinoLED, HIGH);
}

void led_off() 
{
    Serial.println("LED off");
    digitalWrite(arduinoLED, LOW);
}



// Movement with argument commands
void run_engine() 
{
    int new_powers[num_drive_motors];
    int allzero = 0;

    for (int i=0; i < num_drive_motors; i++){
        new_powers[i] = atoi(sCmd.next());
        /* If all zeros, stop motors */
        allzero = allzero || new_powers[i];
        /* Apply correction & direction here */
        new_powers[i] *= driveset[i]->direction;
        new_powers[i] += (int) lround(driveset[i]->correction);
    }

    #ifdef FW_DEBUG
    Serial.print("Moving ");
    for(int i=0; i < num_drive_motors; i++){
        Serial.print(new_powers[i]);
        Serial.print(" ");
    }
    Serial.println();
    #endif

    /* //Stops the motors when the signal given is 0 0 */
    /* if(new_left_power == 0 && new_right_power == 0){ */
    /*     if(!stop_flag) { */
    /*         brake_motors(); */
    /*         stop_flag = 1; */
    /*     } */
    /* } else { */
    /*     stop_flag = 0; */
    /* } */

    if(allzero) {
        if(!
    }
    

    function_running = 0;
    

    //Checks if the given speed is less than the minimum speed. If it is, 
    //it sets the given power to the minimum power. Left motor.
    if ((new_left_power != 0) && (abs(new_left_power) < minpower)) {
        new_left_power = minpower * (new_left_power/abs(new_left_power)); // ooooh
    }

    if ((new_right_power != 0) && (abs(new_right_power) < minpower)) {
        new_right_power = minpower  * (new_right_power/abs(new_right_power));
    }

    // Updates speed of right wheel motor only if a different power is given.
    if(new_right_power != right_power){
        right_power = new_right_power;
        Serial.print("Changing right power to ");
        Serial.println(right_power);
        if(right_power < 0){
            motorBackward(rightm, abs(right_power));
        } 
        else {
            motorForward(rightm, right_power);
        }
    }

    // Updates speed of left wheel motor only if a different power is given.
    if(new_left_power != left_power){
        left_power = new_left_power;
        Serial.print("Changing left power to ");
        Serial.println(left_power);
        if(left_power < 0){
            motorBackward(leftm, abs(left_power));
        } 
        else {
            motorForward(leftm, left_power);
        }
    }
}


// Kick script

void kick() 
{
    kicking = 1;
}

void move_kick() 
{
    switch(kick_state) {
    case(0):
        function_running = 0; // Set as a non scripted function. Is still blocking currently, may be changed
      
        kick_interval = millis();

        leftStop();
        rightStop();

        Serial.println("Kicking");

        motorBackward(catcher, 100);
        break;
        //delay(300);
    case(1):
        motorForward(kicker, 100);
        motorForward(leftm, 70);
        motorForward(rightm, 70);

        break;
        //delay(time);
    case(2):
        motorBackward(kicker, 100);
        motorBackward(catcher, 100);
        brake_motors();
        break;
    case(3):
        motorStop(kicker);
        motorStop(catcher);
        kicking = 0;
        break;
    }

}


// Moves the catcher to a catching position
void move_catchup() 
{
    Serial.println("Catcher on");
    //lift and move forward
    motorBackward(catcher, 100);

}

// Stops lifting the catcher so it falls on the ball
void move_catchdown() 
{
    Serial.println("Catcher off");
    //lift and move forward
    motorStop(catcher);

}

// Function to stop left motor, also sets the speed to 0
// (These are used so I don't have to copy and paste this code everywhere)
void leftStop() 
{
    left_power = 0;
    motorStop(leftm);
}

// Function to stop right motor, also sets the speed to 0
void rightStop() 
{
    right_power = 0;
    motorStop(rightm);
}

/* Force stops all motors */
void force_stop()
{
    Serial.println("Force stopping");
    for (int i=0; i < num_drive_motors; i++){
        driveset[i]->power = 0;
    }
    motorAllStop(); 
}

//Script to rotate the robot quickly to the left for a specified time.
void move_shortrotL() 
{
    if(function_running != 1) {
        char *arg1;
        char *arg2;

        arg1 = sCmd.next();
        arg2 = sCmd.next();

        if (arg1 != NULL) {
            interval = atoi(arg1);
        } 
        else
        {
            interval = 250;
        }

        if (arg2 != NULL) {
            srot_power = atoi(arg2);
        }
        else
        {
            srot_power = default_power;
        }

        motorForward(rightm, srot_power);
        motorBackward(leftm, srot_power);
        stop_flag = 0;

        function_running = 1;
        function_run_time = millis();
    }
}

//Script to rotate the robot quickly to the right for a specified time.
void move_shortrotR() 
{
    if(function_running != 2) {
        char *arg1;
        char *arg2;

        arg1 = sCmd.next();
        arg2 = sCmd.next();

        if (arg1 != NULL) {
            interval = atoi(arg1);
        } 
        else
        {
            //Left motor slightly weaker forwards, compensated here
            interval = 300;
        }

        if (arg2 != NULL) {
            srot_power = atoi(arg2);
        }
        else
        {
            srot_power = default_power;
        }

        motorForward(leftm, srot_power);
        motorBackward(rightm, srot_power);
        stop_flag = 0;

        function_running = 2;
        function_run_time = millis();
    }

}


//Remote Control Commands
void rc_forward() 
{
    Serial.println("Moving forward");
    motorForward(4, 100);
    motorForward(5, 100);

}

void rc_backward()
{
    Serial.println("Moving backward");
    motorBackward(4, 100);
    motorBackward(5, 100);

}

void rc_rotateL() 
{
    Serial.println("Rotating Left");
    motorForward(4, 100);
    motorBackward(5, 100);
}

void rc_rotateR() 
{
    Serial.println("Rotating Right");
    motorForward(5, 100);
    motorBackward(4, 100);
}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) 
{
    Serial.println("I'm sorry, Dave. I'm afraid I can't do that.");
}

void brake_motors()
{
    left_power = 0;
    right_power = 0;
    function_running = 1;
    interval = 100;
    function_run_time = millis();
  
    motorBrake(leftm, 100);
    motorBrake(rightm, 100);
}
