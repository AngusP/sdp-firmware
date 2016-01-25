#include "CommandSet.h"
#include "State.h"
#include "SerialCommand.h"
#include "SDPArduino.h"
#include "Adafruit_NeoPixel.h"

// TODO move this elsewhere
// ALL THE LEDEEES!
#define NUM_PIXELS 10
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, 5, NEO_GRB + NEO_KHZ800);

static SerialCommand sCmd; // The SerialCommand object


// TODO move this elsewhere
void write_powers()
{
    for(int i=0; i < motor_count; i++){
        #ifdef FW_DEBUG
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(state.motors[i]->power);
        Serial.print(F(" "));
        #endif
        if(state.motors[i]->power < 0){
            motorBackward(state.motors[i]->port, abs(state.motors[i]->power));
        } else {
            motorForward(state.motors[i]->port,  abs(state.motors[i]->power));
        }
    }
    #ifdef FW_DEBUG
    Serial.println();
    #endif
}


void CommandSet::setup()
{
    /* Setup callbacks for SerialCommand commands */
    sCmd.addCommand("L", this->led);              // Toggles LED
    sCmd.addCommand("ping", this->ping);          // Check serial link
    sCmd.addCommand("help", this->help);

    /* Movement commands */
    sCmd.addCommand("M", this->move);             // Runs wheel motors
    sCmd.addCommand("S", this->stop);             // Force stops all motors
    sCmd.addCommand("G", this->go);               // Runs wheel motors with correction

    /* Read from rotary encoders */
    sCmd.addCommand("Speeds", this->speeds);

    /* Misc commands */
    sCmd.addCommand("Recv", this->receive);       // Milestone 1
    sCmd.addCommand("Pixels", this->pixels);      // Set LED colour

    sCmd.setDefaultHandler(this->unrecognized);   // Handler for command that isn't matched


    // TODO move this
    strip.begin();
    strip.show();
}

void CommandSet::readSerial()
{
    // Poll serial buffer & check against command set
    sCmd.readSerial();
}

/************************************* Setup callbacks for SerialCommand commands *************************************/

void CommandSet::led()
{
    state.status_led = !state.status_led;
    if (state.status_led){
        digitalWrite(state.status_led_pin, LOW);
    } else {
        digitalWrite(state.status_led_pin, HIGH);
    }
}

void CommandSet::ping()
{
    Serial.println(F("pong"));
}

void CommandSet::help()
{
    Serial.println(F("Valid input commands: (some have arguments)"));
    sCmd.dumpCommandSet();
}

/****************************************** Movement with argument commands *******************************************/

void CommandSet::move()
{
    int new_powers[motor_count];
    int allzero = 0;

    for (int i=0; i < motor_count; i++){
        new_powers[i] = atoi(sCmd.next());
        /* If all zeros, stop motors */
        allzero = allzero || new_powers[i];
        /* Apply correction & direction here */
        new_powers[i] *= state.motors[i]->direction;
    }

    #ifdef FW_DEBUG
    Serial.println(F("Moving"));
    #endif

    if(allzero)
        brake_motors();

    /* update speeds of all drive motors */
    for(int i=0; i < motor_count; i++){
        state.motors[i]->power = new_powers[i];
    }
    write_powers();
}

void CommandSet::stop()
{
    #ifdef FW_DEBUG
    Serial.println(F("Force stopping"));
    #endif

    for (int i=0; i < motor_count; i++){
        state.motors[i]->power = 0;
    }
    motorAllStop();
}

void CommandSet::go()
{

}

/********************************************* Read from rotary encoders **********************************************/

void CommandSet::speeds()
{
    for (int i = 0; i < motor_count; i++) {
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(state.motors[i]->speed);
        Serial.print(F(" "));
    }
    Serial.println();
}

/*************************************************** Misc commands ****************************************************/

/*
  Change state to waiting for up-to 250 bytes from controller

  Comms syntax:
  Recv (int:send speed) (int:number of bytes)
*/
void CommandSet::receive()
{
    char* arg1 = sCmd.next();
    char* arg2 = sCmd.next();
    bool die = false;

    arg1 == NULL ? die = true : state.send_frequency = atoi(arg1);
    arg2 == NULL ? die = true : state.num_bytes      = atoi(arg2);

    if (die) {
        return CommandSet::unrecognized(NULL);
    }

    state.num_bytes = state.num_bytes > 250 ? 250 : state.num_bytes;

    state.receiving = true;
}

void CommandSet::pixels()
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

    for (int i=0; i<NUM_PIXELS; i++) {
        strip.setPixelColor(i, red, green, blue);
    }
    strip.show();
}

/*
 * This gets set as the default input handler, and gets called when no other command matches.
 */
void CommandSet::unrecognized(const char* command)
{
    /* NACK */
    Serial.println(F("N"));
}
