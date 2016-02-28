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
    sCmd.addCommand("L",       this->led);          // Toggles LED
    sCmd.addCommand("ping",    this->ping);         // Check serial link
    sCmd.addCommand("help",    this->help);

    /* Movement commands */
    sCmd.addCommand("M",       this->move);         // Runs wheel motors
    sCmd.addCommand("S",       this->stop);         // Force stops all motors
    sCmd.addCommand("G",       this->go);           // Runs wheel motors with correction

    /* Read from rotary encoders */
    sCmd.addCommand("speeds",  this->speeds);

    /* Misc commands */
    sCmd.addCommand("pixels",  this->pixels);       // Set LED colour

    sCmd.addCommand("grab",    this->grab);         // Grab 0 or 1
    sCmd.addCommand("kick",    this->kick);         // Kick

    sCmd.addCommand("rotate",  this->rotate);       // Rotate

    /* Debug and inspection commands */

    sCmd.addCommand("ps",      this->proc_dump);    // Show process status
    sCmd.addCommand("ptog",    this->proc_toggle);  // Enable or disable by pid on the fly
    
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
    Serial.print(F("N - "));
    Serial.println(command);
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
    delay(10);
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

    Serial.println(F("A"));
    #ifdef FW_DEBUG
    Serial.println(F("Moving"));
    #endif
        
    /* update speeds of all drive motors */
    for(int i=0; i < motor_count; i++){
        state.motors[i]->power = state.motors[i]->desired_power = new_powers[i];
        state.motors[i]->disp_delta = 0;
    }
    write_powers();
}

void CommandSet::stop()
{
    Serial.println(F("A"));
    #ifdef FW_DEBUG
    Serial.println(F("Force stopping"));
    #endif

    write_powers(0);
    motorAllStop();
}

void CommandSet::go()
{
    /* holonomics and shit */
    float x_vel = atof(sCmd.next());
    float y_vel = atof(sCmd.next());
    float r_vel = atof(sCmd.next());

    int power   = atoi(sCmd.next());
    power = power > 0 && power <= 255 ? power : 255;

    Serial.println(F("A"));
    #ifdef FW_DEBUG
    Serial.println(F("Going"));
    #endif

    float new_powers[motor_count];

    new_powers[0] =
        state.velo_coupling_mat[0][0] * x_vel +
        state.velo_coupling_mat[0][1] * y_vel +
        state.velo_coupling_mat[0][2] * r_vel;

    new_powers[1] =
        state.velo_coupling_mat[1][0] * x_vel +
        state.velo_coupling_mat[1][1] * y_vel +
        state.velo_coupling_mat[1][2] * r_vel;

    new_powers[2] =
        state.velo_coupling_mat[2][0] * x_vel +
        state.velo_coupling_mat[2][1] * y_vel +
        state.velo_coupling_mat[2][2] * r_vel;

    float largest = 0.0;
    for (int i=0; i<3; i++){
        if (largest < fabs(new_powers[i]))
            largest = fabs(new_powers[i]);
    }
    
    for (int i=0; i<3; i++){
        new_powers[i] = round(new_powers[i] * (1.0/largest) * (float) power);
    }

    for(int i=0; i < motor_count; i++){
        state.motors[i]->power = state.motors[i]->desired_power = (int) new_powers[i];
        state.motors[i]->disp_delta = 0;
    }
    write_powers();
    
}

/***  READ ROTARY ENCODERS  **************************************************************************/

void CommandSet::speeds()
{
    Serial.print(millis());
    Serial.print(F("ms: "));
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

    Serial.println(F("A"));
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

    if (state.kg_handler_action != -1) {
        Serial.println(F("N - grab"));
        return;
    }

    Serial.println(F("A"));
    #ifdef FW_DEBUG
    Serial.print(F("grabbing "));
    Serial.println(direction);
    #endif

    process* handler = state.kick_grab_handler;
    processes.enable(handler->id);

    if(direction){
        /* close */
        state.kg_handler_action = 3;
        motorBackward(port,  motor_power);
        processes.change(handler->id, 1500L);
    } else {
        /* open */
        state.kg_handler_action = 1;
        motorForward(port, motor_power);
        processes.change(handler->id, 1500L);
    }

    processes.enable(handler->id);
    processes.forward(handler->id);
}


void CommandSet::kick()
{
    if (state.kg_handler_action != -1) {
        Serial.println(F("N - kick"));
        return;
    }

    Serial.println(F("A"));
    #ifdef FW_DEBUG
    Serial.println(F("kicking"));
    #endif

    digitalWrite(6, HIGH);
    
    process* handler = state.kick_grab_handler;
    state.kg_handler_action = 0;

    /* Activate handler and set interval */
    processes.enable(handler->id);
    processes.change(handler->id, 300L);
    processes.forward(handler->id);
    
}


void CommandSet::rotate()
{
    const int motor_power   = atoi(sCmd.next());
    const long delta        = atoi(sCmd.next());

    Serial.println(F("A"));
    #ifdef FW_DEBUG
    Serial.print(F("rotating at "));
    Serial.print(motor_power);
    Serial.print(F(" to "));
    Serial.print(delta);
    Serial.println(F(" stops"));
    #endif

    for (size_t i=0; i < motor_count; i++){
        state.motors[i]->power = motor_power;
        state.initial_displacement[i] = state.motors[i]->disp;
    }

    state.rotation_delta = delta;

    processes.enable((state.rotation_process)->id);

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

