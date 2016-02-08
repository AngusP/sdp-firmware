#ifndef FIRMWARE_PROCESSES_H
#define FIRMWARE_PROCESSES_H

#include <Arduino.h>
#include <Wire.h>

#define DFL_PROCESS_TABLE_SIZE 5

/* Error codes for panics */
#define PROCESS_ERR_OOM 0

/*
 *  Process structure, for defining a clock synchronous process
 */

typedef size_t pid_t;

typedef struct {
    pid_t id;
    unsigned long last_run, interval;
    bool enabled;
    void (*callback)(pid_t);
    const char *label;
} process;

/*  
 *  Process class, manipulates and executes processes
 */

class Processes {

public:
    void setup();
    void run();
    size_t add(process* proc);
    
    process* get_by_id(pid_t pid);
    process* get_by_callback(void (*callback)(pid_t));

    void enable(pid_t pid);
    void disable(pid_t pid);

    void status();
    
    int change(size_t pid, void (*callback)(pid_t), unsigned long interval);
    int change(size_t pid, void (*callback)(pid_t));
    int change(size_t pid, unsigned long interval);
    
protected:
    process** tasks;
    void panic(int error);
    
private:
    size_t num_tasks;               // Number of processes in tasks
    size_t ptable_size;             // Size of space allocated to tasks
    void grow_table(size_t num);

    /* helpers */
    size_t memcheck();

};

extern Processes processes;


#endif //FIRMWARE_PROCESSES_H
