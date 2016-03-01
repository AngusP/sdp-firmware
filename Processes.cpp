
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
    tasks = (process**) malloc(ptable_size * sizeof(process*));
}


void Processes::run()
{
    for (size_t i = 0; i < num_tasks; i++) {
        if (tasks[i]->enabled && millis() >= (tasks[i]->last_run + tasks[i]->interval)){
            tasks[i]->callback(tasks[i]->id);
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
        Serial.println(F("!! PANIC: OOM!"));
        break;

    default:
        Serial.println(F("!! PANIC"));
    }

    /* Crash (AVR specific) */
    while (true) asm volatile("nop\n");
}


/**
   Print UNIX-like process status table
   Thsi is expensive but should be run occasionally
**/
void Processes::status()
{
    Serial.println();
    Serial.println(F("pid \t enable \t interval \t last run \t callback \t label"));
    Serial.println(F("--- \t ------ \t -------- \t -------- \t -------- \t -----"));
    Serial.flush();
    delay(10);

    for (size_t i=0; i<num_tasks; i++) {
        
        Serial.print(tasks[i]->id);
        Serial.print(F("\t "));

        tasks[i]->enabled ? Serial.print(F("true ")) : Serial.print(F("false"));
        Serial.print(F("\t\t "));
        
        Serial.print(tasks[i]->interval);
        Serial.print(F("\t\t "));
        
        Serial.print(tasks[i]->last_run);
        Serial.print(F("\t\t "));

        /* Apparently this cast is fine 0_o */
        Serial.print((uint16_t) tasks[i]->callback, HEX);
        Serial.print(F("\t\t "));

        Serial.print(tasks[i]->label);
        Serial.println();
        Serial.flush();
        delay(10);
    }

    Serial.println(F("--- \t ------ \t -------- \t -------- \t -------- \t -----"));
    Serial.print(F("mem: "));
    Serial.print(memcheck());
    Serial.print(F(" free of "));
    Serial.println(RAMEND - RAMSTART +1, DEC);
    Serial.flush();
    delay(10);
}

/**
   Direct memory access is fun
**/
size_t Processes::memcheck()
{
    size_t stackptr, heapptr;
    stackptr = (size_t) malloc(4);
    heapptr = stackptr;
    free((void*) stackptr);
    stackptr = (size_t)(&stackptr);

    return stackptr - heapptr;
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

    // TODO: Check pointer logic
    tasks[num_tasks] = proc;
    proc->id = num_tasks++;

    /* Process id is the number -1 as we zero index */
    return num_tasks-1; 
}


process* Processes::get_by_id(pid_t pid)
{
    if (pid >= num_tasks)
        return NULL;

    return tasks[pid];
}

/**
   Not particularly safe, but useful
   Get a reference to a process by matching callback
**/
process* Processes::get_by_callback(void (*callback)(pid_t))
{
    size_t num_hits = 0;
    process* found_process;
    process** tasks_p = tasks;
    
    while (tasks_p != NULL){
        if ((*tasks_p)->callback == callback) {
            num_hits++;
            found_process = *tasks_p++;
        }
    }

    return num_hits == 1 ? found_process : NULL;
}


boolean Processes::is_enabled(pid_t pid)
{
    if (pid >= num_tasks) {
        return false;
    } else {
        return tasks[pid]->enabled;
    }
}

/**
   Used to get more memory for storing processes
 **/
void Processes::grow_table(size_t num)
{
    /*
      Reallocate memory for ther process table, panicking if this does not work.
    */
    ptable_size += num;
    process** new_tasks = (process**) realloc(tasks, ptable_size * sizeof(process*));

    if (new_tasks == NULL) panic(PROCESS_ERR_OOM);

    tasks = new_tasks;
}


void Processes::disable(pid_t pid)
{
    tasks[pid]->enabled = false;
}

/**
   Enable a process and reset last_run in an attempt to make it fire immediately; 
   This is not guaranteed if the robot has only just started running
**/

void Processes::enable(pid_t pid)
{
    tasks[pid]->enabled = true;
    tasks[pid]->last_run = 0;
}

/*
 *  Update a process' callback and/or interval by id reference
 *  @return: -1 if failure, 0 if success
 */

int Processes::change(pid_t pid, void (*callback)(pid_t), unsigned long interval)
{
    return change(pid, callback) | change(pid, interval);
}

int Processes::change(pid_t pid, void (*callback)(pid_t))
{
    if (pid >= num_tasks) return -1;
    tasks[pid]->callback = callback;
    return 0;
}

int Processes::change(pid_t pid, unsigned long interval)
{
    if (pid >= num_tasks) return -1;
    tasks[pid]->interval = interval;
    return 0;
}

/*
 *  Set a process' last run to the current time. 
 *  Useful for state-machine like behaviour
 *  @return: -1 if failure, 0 if success
 */

int Processes::forward(pid_t pid) {
    if (pid >= num_tasks) return -1;
    tasks[pid]->last_run = millis();
    return 0;
}

