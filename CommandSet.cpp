#include "CommandSet.h"
#include "Processes.h"
#include "robot.h"
#define FW_DEBUG

// TODO move this elsewhere
// ALL THE LEDEEES!
#define NUM_PIXELS 10
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, 5, NEO_GRB + NEO_KHZ800);

static SerialCommand sCmd; // The SerialCommand object

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
    sCmd.addCommand("speeds", this->speeds);

    /* Misc commands */
    sCmd.addCommand("pixels", this->pixels);      // Set LED colour

    sCmd.addCommand("grab", this->grab);          // Grab 0 or 1
    sCmd.addCommand("kick", this->kick);          // Kick

    sCmd.addCommand("rotate", this->rotate);      // Rotate

    sCmd.addCommand("stall", this->updateStall);

    /* Debug and inspection commands */

    sCmd.addCommand("ps", this->proc_dump);
    sCmd.addCommand("ptog", this->proc_toggle);
    
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

    for (int i=0; i < motor_count; i++){
        new_powers[i] = atoi(sCmd.next());
        /* Apply direction correction here */
        new_powers[i] *= state.motors[i]->direction;
    }

    #ifdef FW_DEBUG
    Serial.println(F("Moving"));
    #endif
        
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

    write_powers(0);
    motorAllStop();
}

void CommandSet::go()
{

}

/***  READ ROTARY ENCODERS  **************************************************************************/

void CommandSet::speeds()
{
    Serial.print(millis());
    Serial.print(F(", "));
    for (int i = 0; i < motor_count; i++) {
        Serial.print(state.motors[i]->speed);
        Serial.print(F(", "));
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



void CommandSet::grab()
{
    int direction = atoi(sCmd.next());
    
    const int port          = 1;
    const int motor_power   = 255;

    #ifdef FW_DEBUG
    Serial.print(F("grabbing "));
    Serial.println(direction);
    #endif

    if(direction){
        /* close */
        motorForward(port,  motor_power);
        delay(1500);
    } else {
        /* open */
        motorBackward(port, motor_power);
        delay(1500);
        motorForward(port, motor_power);
        delay(700);
    }

    motorStop(port);

    #ifdef FW_DEBUG
    Serial.println(F("done "));
    #endif
}


void CommandSet::kick()
{
    #ifdef FW_DEBUG
    Serial.println(F("kicking"));
    #endif

    //TODO
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

    // TODO: Find a safe way to do this
    //processes.change(CHECK_MOTORS_PROCESS, processes.check_rotation, 60);
    //processes.enable(CHECK_MOTORS_PROCESS);

    write_powers();
}

void CommandSet::proc_dump()
{
    processes.status();
}

void CommandSet::proc_toggle()
{
    pid_t pid = (pid_t) atoi(sCmd.next());

    process* proc = processes.get_by_id(pid);

    if (proc == NULL) {
        Serial.print(F("Unknown pid "));
        Serial.println(pid);
        return;
    }
    
    proc->enabled ? processes.disable(pid) : processes.enable(pid);

    Serial.print(F("toggled pid "));
    Serial.println(pid);
}

void CommandSet::updateStall()
{
    float new_threshold = atoi(sCmd.next());

    Serial.print(F("New threshold is "));
    Serial.print(new_threshold);

    state.stall_threshold = new_threshold;
}

