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
void pong();
void dump_commands();
void poll_encoders();
void led_toggle();
void write_powers();
void run_motors();
void brake_motors();
void all_stop();
void motor_stop(struct motor*);
void unrecognised(const char*);
void print_speeds();

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

/*
  Toggle LED ~1 time per second
  999 so it is unlikely to coincide
  with things that actually have to 
  happen once per second
*/
struct process heartbeat = {
    .last_run = 0,
    .interval = 999,
    .callback = &led_toggle
};

struct process update_speeds = {
    .last_run = 0,
    .interval = 50,
    .callback = &poll_encoders
};

struct process* tasks[] = {
    &heartbeat,
    &update_speeds
};

/* 
   Generic motor structure, carries
   around all needed information.
   
   IT IS ASSUMED that the encoders
   are connected in the same order
   that the motors are added to the
   driveset array below.
*/
struct motor {
    int port, power, direction;
    float speed;
};

struct motor holo1 = {
    .port         = 2,
    .power        = 0,
    .direction    = 1,
    .speed        = 0.0
};
struct motor holo2 = {
    .port         = 5,
    .power        = 0,
    .direction    = 1,
    .speed        = 0.0
};
struct motor holo3 = {
    .port         = 4,
    .power        = 0,
    .direction    = 1,
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

    sCmd.dumpCommandSet();
    
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
        time_period = (1.0/(long) send_frequency) * 1000.0;

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
    sCmd.addCommand("L", led_toggle);        // Toggles LED
    sCmd.addCommand("ping", pong);           // Check serial link
    sCmd.addCommand("help", dump_commands);

    /* Movement commands */
    sCmd.addCommand("M", run_motors);        // Runs wheel motors
    sCmd.addCommand("F", all_stop);          // Force stops all motors

    /* Read from rotary encoders */
    sCmd.addCommand("Speeds", print_speeds);

    /* Misc commands */
    sCmd.addCommand("Recv", init_receive);   // Milestone 1
    sCmd.addCommand("Pixels", set_pixels);   // Set LED colour

    sCmd.setDefaultHandler(unrecognized);    // Handler for command that isn't matched
}

void pong()
{
    Serial.println("pong");
}

void dump_commands()
{
    Serial.println("Valid input commands: (some have arguments)");
    sCmd.dumpCommandSet();
}

/*
  Callback for the update_speeds process
*/
void poll_encoders()
{
    Wire.requestFrom(encoder_i2c_addr, num_drive_motors);

    for (int i = 0; i < num_drive_motors; i++) {
        byte delta = (int8_t) Wire.read();
        if (driveset[i]->power < 0) delta = 0xFF - delta;
        /*
          Update speed using the time now and the time we last checked
        */
        driveset[i]->speed = (float) delta /
            ((float) millis() - (float) update_speeds.last_run);
    }
}

void print_speeds()
{
    for (int i = 0; i < num_drive_motors; i++) {
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(driveset[i]->speed);
        Serial.print(F("stops/s "));
    }
    Serial.println();
}

/*
  Test Commands 
*/
void led_toggle() 
{
    self.status_led = !self.status_led;
    if (self.status_led){
        digitalWrite(self.status_led_pin, LOW);
    } else {
        digitalWrite(self.status_led_pin, HIGH);
    }
}

void write_powers()
{
    for(int i=0; i < num_drive_motors; i++){
        #ifdef FW_DEBUG
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(driveset[i]->power);
        Serial.print(F(" "));
        #endif
        if(driveset[i]->power < 0){
            motorBackward(driveset[i]->port, abs(driveset[i]->power));
        } else {
            motorForward(driveset[i]->port,  abs(driveset[i]->power));   
        }
    }
    #ifdef FW_DEBUG
    Serial.println();
    #endif
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
    }

    #ifdef FW_DEBUG
    Serial.println(F("Moving"));
    #endif

    if(allzero)
        brake_motors();

    /* update speeds of all drive motors */
    for(int i=0; i < num_drive_motors; i++){
        driveset[i]->power = new_powers[i];
    }
    write_powers();
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

