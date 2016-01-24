/***
   
    F I R M W A R E

    SDP2016 Group 1

***/

#include "SerialCommand.h"
#include "SDPArduino.h"
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define FW_DEBUG                             // Comment out to remove serial debug chatter

#define sensorAddr 0x39  // Sensor physical address on the power board - 0x39
#define ch0        0x43  // Read ADC for channel 0 - 0x43
#define ch1        0x83  // Read ADC for channel 1 - 0x83

#define encoder_i2c_addr 5

SerialCommand sCmd;                         // The SerialCommand object

// ALL THE LEDEEES!
#define NUM_PIXELS 5
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, 5, NEO_GRB + NEO_KHZ800);
// Ards are Num pixels, Pin and other stuff

/***
    STRUCTURES & PROTOTYPES
***/
void init_commandset();
void updateMotorPositions();
void printMotorPositions();
void getenc();
void led_toggle();
void run_motors();
void brake_motors();
void all_stop();
void motor_stop(struct motor*);
void unrecognised(const char*);

/* milestone hoops: */
void init_receive();
byte receive_bytes[250];
int send_frequency;
int num_bytes;
int sending_index;
bool receiving = false;
bool sending = false;
unsigned long last_send;
long time_period; 

/* 
   Hold state variables
*/
struct state {
    int battery, status_led, status_led_pin;
    float heading;
};

struct state self = {
    .battery = 100,
    .status_led = 0,
    .status_led_pin = 13,
    .heading = 0.0
};

/* 
   Process structure, for defining a 
   clock synchronous process 
*/
struct process {
    unsigned long last_run, interval;
    void (*callback)();
};

/*struct process test = {
    .last_run = 0,
    .interval = 1000,
    .callback = &testfunc
};*/

struct process* tasks[] = {
    //&test
};


/* 
   Sensor information, may be replaced
   with specific classes per sensor
*/
struct sensor {
    int i2c_addr, last_val;
};

struct sensor encoders = {
    .i2c_addr = 0x05,
    .last_val = 0
};

/* 
   Generic motor structure, carries
   around all needed information
*/
struct motor {
    int port, encoder, power, direction;
    float correction, speed;
};

struct motor holo1 = {
    .port         = 2,
    .encoder      = 0,
    .power        = 0,
    .direction    = 1,
    .correction   = 0.0,
    .speed        = 0.0
};
struct motor holo2 = {
    .port         = 5,
    .encoder      = 1,
    .power        = 0,
    .direction    = 1,
    .correction   = 0.0,
    .speed        = 0.0
};
struct motor holo3 = {
    .port         = 4,
    .encoder      = 2,
    .power        = 0,
    .direction    = 1,
    .correction   = 0.0,
    .speed        = 0.0
};

/* 
   Define an array of our drive motors,
   this generalises the motor control logic
   
   NB this is *not* itself a struct
*/
struct motor* driveset[] = {
    &holo1,
    &holo2,
    &holo3
};
const int num_drive_motors = sizeof(driveset)/sizeof(struct motor*);

// Rotary encoders
int positions[num_drive_motors] = {0};


/***
    LOOP & SETUP
***/
void setup()
{
    pinMode(self.status_led_pin, OUTPUT);
    digitalWrite(self.status_led_pin, self.status_led);

    SDPsetup();

    Serial.begin(115200);
    Serial.println(F("STARTUP"));

    init_commandset();

    strip.begin();
    strip.show();

    for (int i=0; i<5; i++)
        strip.setPixelColor(i, 255, 255, 255);
    strip.show();
}

void loop() 
{ 
    
    /* Poll serial buffer & check against command set */
    sCmd.readSerial();


    /* Run through schedule */
    for (int i=0; i<(sizeof(tasks)/sizeof(struct process*)); i++){
        if (millis() >= (tasks[i]->last_run + tasks[i]->interval)){
            tasks[i]->callback();
            tasks[i]->last_run = millis();
        }
    }
  

    /* Milestone 1 */

    if (receiving){
        sending_index = 0;

        #ifdef FW_DEBUG
        Serial.println(F("Waiting for bytes..."));
        #endif
        
        /* Block and read serial */
        while (sending_index < num_bytes){
            if(Serial.available()){
                receive_bytes[sending_index++] = (byte) Serial.read();

                #ifdef FW_DEBUG
                Serial.print(F("Read 0x"));
                Serial.println(receive_bytes[sending_index-1], HEX);
                #endif
            }
        }
        sending = true;
        receiving = false;
        sending_index = 0;
        last_send = millis();
        time_period = (1.0/(long) send_frequency) * 1000.0; // *1000 because millis not seconds

        #ifdef FW_DEBUG
        Serial.println(F("Got it, thanks bub"));
        Serial.print(F("Writing at "));
        Serial.print(send_frequency);
        Serial.print(F(" ("));
        Serial.print(time_period);
        Serial.println(F(" s^-1)"));
        #endif
    }

    // Ewww.
    if (sending && (millis() >= (last_send + time_period))){
        if (sending_index < num_bytes) {
            // 0x45 is the address according to the milestone page
            Wire.beginTransmission(0x45);
            
            #ifdef FW_DEBUG
            Serial.print(F("Writing 0x"));
            Serial.print(receive_bytes[sending_index], HEX);
            Serial.println(F(" to bus"));
            #endif

            Wire.write(receive_bytes[sending_index++]);
            Wire.endTransmission();
        } else {
            sending = false;
            
            #ifdef FW_DEBUG
            Serial.print(F("Sent "));
            Serial.print(num_bytes);
            Serial.println(F(" bytes over the i2c bus"));
            #endif
        }
        last_send = millis();
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

    /* Read from rotary encoders */
    sCmd.addCommand("GETE", getenc);

    /* Misc commands */
    sCmd.addCommand("Recv", init_receive);
    sCmd.addCommand("Pixels", set_pixels);  // Set LED colour

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
    Serial.print(F("Encoders: "));
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

// Test Commands
void led_toggle() 
{
    self.status_led = !self.status_led;
    if (self.status_led){
        Serial.println(F("LED off"));
        digitalWrite(self.status_led_pin, LOW);
    } else {
        Serial.println(F("LED on"));
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
    Serial.print(F("Moving "));
    for(int i=0; i < num_drive_motors; i++){
        Serial.print(new_powers[i]);
        Serial.print(" ");
    }
    Serial.println();
    #endif

    if(allzero)
        brake_motors();
    
    /* 
       We could have logic here to prevent stalling and high current
       draw, but given we've corrected the motors' speeds we can't
       reliably check this yet so that's a TODO...
    */

    /* update speeds of all drive motors */
    for(int i=0; i < num_drive_motors; i++){
        driveset[i]->power = new_powers[i];
        #ifdef FW_DEBUG
        Serial.print(F("Changing power "));
        Serial.print(i);
        Serial.print(F(" to "));
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
    Serial.println(F("Force stopping"));
    #endif
    
    for (int i=0; i < num_drive_motors; i++){
        driveset[i]->power = 0;
    }
    motorAllStop();
}

void set_pixels()
{
    byte red   = atoi(sCmd.next());
    byte green = atoi(sCmd.next());
    byte blue  = atoi(sCmd.next());

    #ifdef FW_DEBUG
    Serial.print(F("Set pixel colour to "));
    Serial.print(red,HEX);
    Serial.print(green,HEX);
    Serial.println(blue,HEX);
    #endif
    
    for (int i=0; i<NUM_PIXELS; i++)
        strip.setPixelColor(i, red, green, blue);
    strip.show();
}


/*
This gets set as the default input handler, and gets 
called when no other command matches.
*/
void unrecognized(const char* command)
{
    /* NACK */
    Serial.println(F("N"));
}

/** MILESTONE 1 **/

/*
  Change state to waiting for up-to 250 bytes from controller
 
  Comms syntax:
  Recv (int:send speed) (int:number of bytes)
*/
void init_receive()
{
    char* arg1 = sCmd.next();
    char* arg2 = sCmd.next();
    bool die = false;
    
    arg1 == NULL ? die = true : send_frequency = atoi(arg1);
    arg2 == NULL ? die = true : num_bytes      = atoi(arg2);

    if(die) return unrecognized(NULL);

    num_bytes = num_bytes > 250 ? 250 : num_bytes;

    receiving = true;
}

