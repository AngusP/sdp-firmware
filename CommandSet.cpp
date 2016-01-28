#include "CommandSet.h"
#include "Processes.h"
#define FW_DEBUG

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
    sCmd.addCommand("Pixels", this->pixels);      // Set LED colour

    sCmd.addCommand("kick", this->kick);          // Kick

    sCmd.addCommand("rotate", this->rotate);      // Rotate

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

/*
 * This gets set as the default input handler, and gets called when no other command matches.
 */
void CommandSet::unrecognized(const char* command)
{
    /* NACK */
    Serial.println(F("N"));
}

/***  SETUP CALLBACKS FOR SERIAL COMMANDS  ***********************************************************/

void CommandSet::led()
{
    digitalWrite(state.status_led_pin, state.status_led = state.status_led ? LOW : HIGH);
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

/***  LOW LEVEL MOTOR COMMAND  ***********************************************************************/

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

/***  READ ROTARY ENCODERS  **************************************************************************/

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

/***  MISC COMMANDS  *********************************************************************************/


void CommandSet::pixels()
{
    byte red   = (byte) atoi(sCmd.next());
    byte green = (byte) atoi(sCmd.next());
    byte blue  = (byte) atoi(sCmd.next());

    #ifdef FW_DEBUG
    Serial.print(F("Set pixel colour to "));
    Serial.print(red,HEX);
    Serial.print(green,HEX);
    Serial.println(blue,HEX);
    #endif

    for (uint16_t i = 0; i < NUM_PIXELS; i++) {
        strip.setPixelColor(i, red, green, blue);
    }
    strip.show();
}



void CommandSet::kick()
{
    const int port          = 1;
    int motor_power         =            atoi(sCmd.next());
    unsigned int duration   = (unsigned) atoi(sCmd.next());

    #ifdef FW_DEBUG
    Serial.print(F("Trying to kick at "));
    Serial.print(motor_power);
    Serial.print(F(" power for "));
    Serial.print(duration);
    Serial.println(" milliseconds. Goodbye.");
    #endif

    if(motor_power < 0){
        motorBackward(port, abs(motor_power));
    } else {
        motorForward(port,  abs(motor_power));
    }

    delay(duration);

    motorStop(port);
}


void CommandSet::rotate()
{
    const int motor_power   = atoi(sCmd.next());
    const long delta        = atoi(sCmd.next());

    #ifdef FW_DEBUG
    Serial.print(F("Going to rotate at "));
    Serial.print(motor_power);
    Serial.print(F(" power until we rotate "));
    Serial.print(delta);
    Serial.println(" units.");
    #endif

    for (size_t i=0; i < motor_count; i++){
        state.motors[i]->power = motor_power;
        state.initial_displacement[i] = state.motors[i]->disp;
    }

    state.rotation_delta = delta;

    processes.change(CHECK_MOTORS_PROCESS, processes.check_rotation, 60);
    processes.enable(CHECK_MOTORS_PROCESS);

    write_powers();
}
