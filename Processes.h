#ifndef FIRMWARE_PROCESSES_H
#define FIRMWARE_PROCESSES_H

#include <Arduino.h>
#include <Wire.h>

#define PROCESS_COUNT 3
#define HEARTBEAT_PROCESS 0
#define POLL_ENCODERS_PROCESS 1
#define CHECK_MOTORS_PROCESS 2

#define DFL_PROCESS_TABLE_SIZE 5

/* Error codes for panics */
#define PROCESS_ERR_OOM 0

/*
 *  Process structure, for defining a clock synchronous process
 */

typedef struct {
    size_t id,
    unsigned long last_run, interval;
    bool enabled;
    void (*callback)();
} process;

/*  
 *  Process class, manipulates and executes processes
 */

class Processes
{

    public:
        void setup();
        void add(process* proc);
        static void run();
        static void enable(size_t process_id);
        static void disable(size_t process_id);

        static void change(size_t process_id, void (*callback)(), unsigned long interval);
        static void change(size_t process_id, void (*callback)());
        static void change(size_t process_id, unsigned long interval);

        static void heartbeat();
        static void poll_encoders();
        static void check_motors();
        static void milestone_1();
        static void check_rotation();

    private:
        process* tasks;
        size_t num_tasks;               // Number of processes in tasks
        size_t ptable_size;             // Size of space allocated to tasks
        void grow_table(size_t num);
        static void panic(int error);
};

extern Processes processes;


#endif //FIRMWARE_PROCESSES_H
