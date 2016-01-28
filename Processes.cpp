
#define FW_DEBUG

#include "Processes.h"
#include "CommandSet.h"
#include "State.h"
#include "addresses.h"

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

    /* Crash (AVR specific) */
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
        grow_table(num_tasks - ptable_size +1);

    tasks[num_tasks++] = proc;
    proc->id = num_tasks;

    /* Process id is the number -1 as we zero index */
    return num_tasks-1;
}

void Processes::grow_table(size_t num)
{
    /*
      Reallocate memory for ther process table, panicking if this does not work.
    */
    ptable_size += num;
    process* new_tasks = (process*) realloc(tasks, ptable_size * sizeof(process));
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
 *  Update a process' callback and/or interval by id reference
 *  @return: -1 if failure, 0 if success
 */

int Processes::change(size_t pid, void (*callback)(), unsigned long interval)
{
    return change(pid, callback) | change(pid, interval);
}

int Processes::change(size_t pid, void (*callback)())
{
    if (pid >= num_tasks) return -1;
    tasks[pid]->callback = callback;
    return 0;
}

int Processes::change(size_t pid, unsigned long interval)
{
    if (pid >= num_tasks) return -1;
    tasks[pid]->interval = interval;
    return 0;
}
