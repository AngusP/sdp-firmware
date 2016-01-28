
#define FW_DEBUG

#include "Processes.h"
#include "CommandSet.h"
#include "State.h"
#include "addresses.h"

/*
  Toggle LED ~1 time per second; 999 so it is unlikely to coincide
  with things that actually have to happen once per second
*/
/*
static process heartbeat_process = {
    .id         = HEARTBEAT_PROCESS,
    .last_run   = 0,
    .interval   = 999,
    .enabled    = true,
    .callback   = &Processes::heartbeat
};

static process poll_encoders_process = {
    .id         = POLL_ENCODERS_PROCESS,
    .last_run   = 0,
    .interval   = 50,
    .enabled    = true,
    .callback   = &Processes::poll_encoders
};

static process check_motors_process = {
    .id         = CHECK_MOTORS_PROCESS,
    .last_run   = 0,
    .interval   = 50,
    .enabled    = false,
    .callback   = &Processes::check_motors
};

process* Processes::collection[PROCESS_COUNT];
*/

void Processes::setup()
{
    /* 
       Dynamically allocate our array of processes. This is 
       risky on an Arduino with it's meagre 2kb of SRAM, 
       but it makes registering of processes cleaner,
       at the expense of setup time.

       Additionally it is important that none of the processes
       malloc themselves memory whilst registering, as calls
       to realloc to grow the process table will have to go
       around that memory, wasting heap space.
    */
    num_tasks = 0;
    ptable_size = DFL_PROCESS_TABLE_SIZE;
    tasks = (process*) malloc(ptable_size * sizeof(process));

    #ifdef FW_DEBUG
    Serial.print(F("Process table initialized at addr 0x"));
    Serial.print(tasks, HEX);
    Serial.print(F(" with size "));
    Serial.println(ptable_size);
    #enif
}

void Processes::run()
{
    for (size_t i = 0; i < num_tasks; i++) {
        if (tasks[i]->enabled && millis() >= (tasks[i]->last_run + tasks[i]->interval)){
            tasks[i]->callback();
            tasks[i]->last_run = millis();
        }
    }
}

/**
   Crash if we cannot safely continue
**/
void Processes::panic(int error)
{
    switch (error) {

    case PROCESS_ERR_OOM:
        Serial.println(F("!! PANIC: Out of memory!"));
        break;

    default:
        Serial.println(F("!! PANIC"));

    }

    /* Crash (AVR Instruction Set) */
    while (true) asm volatile("nop\n");
}


/***  PROCESS MANAGEMENT ROUTINES  *******************************************************************/


size_t Processes::add(process* proc)
{
    /*
      Register a new process. The process struct should have already been allocated.
      First process is 0, we always increment from there on.
      @return: index of process in the process table
    */
    if (num_tasks >= ptable_size)
        grow_table(1);

    tasks[++num_tasks] = proc;
    proc->id = num_tasks;
    return num_tasks;
}

void Processes::grow_table(size_t num)
{
    ptable_size += num;
    new_tasks = (process*) realloc(tasks, ptable_size * sizeof(process));
    new_tasks == NULL ? panic(PROCESS_ERR_OOM) : tasks = new_tasks;
}


void Processes::disable(size_t pid)
{
    tasks[pid]->enabled = false;
}

void Processes::enable(size_t pid)
{
    tasks[pid]->enabled = true;
}

/*
 *  Update a process' callback &| interval by id reference
 */

void Processes::change(size_t pid, void (*callback)(), unsigned long interval)
{
    change(pid, callback);
    change(pid, interval);
}

void Processes::change(size_t pid, void (*callback)())
{
    tasks[pid]->callback = callback;
}

void Processes::change(size_t pid, unsigned long interval)
{
    tasks[pid]->interval = interval;
}


/***  ACTAUAL PROCESS HANDLERS  **********************************************************************/

void Processes::heartbeat()
{
    // Toggles the LED
    command_set.led();
}

void Processes::poll_encoders()
{
    Wire.requestFrom(encoder_i2c_addr, motor_count);

    for (int i = 0; i < motor_count; i++) {
        byte delta = (byte) Wire.read();
        if (state.motors[i]->power < 0) {
            if (delta) {
                delta = 0xFF - delta;
            }
            state.motors[i]->disp -= delta;
        } else {
            state.motors[i]->disp += delta;
        }
        /*
          Update speed using the time now and the time we last checked
        */

        state.motors[i]->speed = (float) delta /
                                 (((float) millis() -
                                   (float) collection[POLL_ENCODERS_PROCESS]->last_run) / 1000.0);

        if (state.motors[i]->power < 0) {
            state.motors[i]->speed *= -1;
        }
    }
}

void Processes::check_motors()
{

}


void Processes::check_rotation()
{
    long current_delta = 0;

    for (size_t i = 0; i < motor_count; i++) {
        current_delta += abs(state.motors[i]->disp - state.initial_displacement[i]);
    }

    current_delta /= motor_count;

    if (current_delta >= state.rotation_delta) {
        #ifdef FW_DEBUG
        Serial.println("Ending rotation");
        #endif
        motorAllStop();
        processes.disable(CHECK_MOTORS_PROCESS);
    }
}
