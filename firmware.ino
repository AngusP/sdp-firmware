/***
   
    F I R M W A R E

    SDP2016 Group 1

***/

#include "SerialCommand.h"
#include "SDPArduino.h"
#include <Wire.h>
#include <Arduino.h>

#define FW_DEBUG                             // Comment out to remove serial debug chatter

#define sensorAddr 0x39  // Sensor physical address on the power board - 0x39
#define ch0        0x43  // Read ADC for channel 0 - 0x43
#define ch1        0x83  // Read ADC for channel 1 - 0x83

#define encoder_i2c_addr 5

SerialCommand sCmd;                         // The SerialCommand object

int stop_flag = 0;

//These global variables are used to keep track of the short rotate function
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
void updateMotorPositions();
void printMotorPositions();
void getenc();
void readI2C(int, int);
void have_ball();
void reset_have_ball();
void led_toggle();
void run_motors();
void brake_motors();
void all_stop();
void motor_stop(struct motor*);
void unrecognised(const char *);


struct state {
    int battery, status_led, status_led_pin;
    float heading;
};

struct state self;

struct motor {
    int port, encoder, power, direction;
    float correction, speed;
};

struct motor holo1;
struct motor holo2;
struct motor holo3;

/* 
   Define an array of our drive motors,
   this generalises the motor control logic

   NB this is *not* itself a struct
*/
const int num_drive_motors = 3;
struct motor* driveset[num_drive_motors] = {
    &holo1,
    &holo2,
    &holo3
};

struct sensor {
    int i2c_addr, last_value;
};

struct sensor encoders;

// Rotary encoders
int positions[num_drive_motors] = {0};


/***
    LOOP & SETUP
***/
void setup()
{
    self.status_led_pin = 13;
    pinMode(self.status_led_pin, OUTPUT);
    digitalWrite(self.status_led_pin, self.status_led);

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
    holo1.speed = 0.0;

    holo2.port = 4;
    holo2.encoder = 1;
    holo2.direction = 1;
    holo2.power = 0;
    holo2.correction = 0.0;
    holo2.speed = 0.0;

    holo3.port = 2;
    holo3.encoder = 2;
    holo3.direction = 1;
    holo3.power = 0;
    holo3.correction = 0.0;
    holo3.speed = 0.0;
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
            all_stop();
            function_running = 0;
        }
        break;
    case 2:
        if (millis() > function_run_time + interval) {
            all_stop();
            function_running = 0;
        }
        break;

    default :
        Serial.println("I shouldn't be here");
    }

    /* poll serial buffer & check against command set */
    sCmd.readSerial();
  
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
    /* Setup callbacks for SerialCommand commands */
    sCmd.addCommand("L", led_toggle);          // Toggles LED

    /* Movement commands */
    sCmd.addCommand("M", run_motors);       // Runs wheel motors
    sCmd.addCommand("F", all_stop);      // Force stops all motors

    sCmd.addCommand("HAVEBALL", have_ball);    //Checks if we have the ball
    sCmd.addCommand("RESETHB", reset_have_ball); //Resets the intial value of the inital light sensor value

    /* Read from rotary encoders */
    sCmd.addCommand("GETE", getenc);

    sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched
}

void updateMotorPositions()
{
    /* Request motor position deltas from encoder board */
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
void led_toggle() 
{
    self.status_led = !self.status_led;
    if (self.status_led){
        Serial.println("LED off");
        digitalWrite(self.status_led_pin, LOW);
    } else {
        Serial.println("LED on");
        digitalWrite(self.status_led_pin, HIGH);
    }
}


// Movement with argument commands
void run_motors() 
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

    if(allzero) {
        if(!stop_flag){
            brake_motors();
            stop_flag = 1;
        }
    } else {
        stop_flag = 0;
    }
    

    function_running = 0;
    
    /* 
       We could have logic here to prevent stalling and high current
       draw, but given we've corrected the motors' speeds we can't
       reliably check this so that's a TODO...
    */

    /* update speeds of all drive motors */
    for(int i=0; i < num_drive_motors; i++){
        driveset[i]->power = new_powers[i];
        #ifdef FW_DEBUG
        Serial.print("Changing power ");
        Serial.print(i);
        Serial.print(" to ");
        Serial.println(new_powers[i]);
        #endif
        if(new_powers[i] < 0){
            motorBackward(driveset[i]->port, abs(new_powers[i]));
        } else {
            motorForward(driveset[i]->port, abs(new_powers[i]));   
        }
    }
}

void brake_motors()
{
    function_running = 1;
    interval = 100;
    function_run_time = millis();
    
    for(int i=0; i < num_drive_motors; i++){
        driveset[i]->power = 0;
        motorBrake(driveset[i]->port, 100);
    }
}


// Function to stop specific motor, also sets the power to 0
void motor_stop(struct motor* m) 
{
    m->power = 0;
    motorStop(m->port);
}

/* Force stops all motors */
void all_stop()
{
    #ifdef FW_DEBUG
    Serial.println("Force stopping");
    #endif
    
    for (int i=0; i < num_drive_motors; i++){
        driveset[i]->power = 0;
    }
    motorAllStop();
}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char* command)
{
    /* NACK */
    Serial.println("N");
}
