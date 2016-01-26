#ifndef FIRMWARE_PROCESSES_H
#define FIRMWARE_PROCESSES_H

#include <Arduino.h>


#define HEARTBEAT_PROCESS 0
#define POLL_ENCODERS_PROCESS 1
#define CHECK_MOTORS_PROCESS 2
#define MILESTONE_1_PROCESS 3

#define PROCESS_COUNT 4

/*
   Process structure, for defining a clock synchronous process
*/
struct process {
    unsigned long last_run, interval;
    bool enabled;
    void (*callback)();
};



class Processes
{

    public:
        void setup();

        static void run();
        static void enable_process(size_t process_id);
        static void disable_process(size_t process_id);

        static void change_process(size_t process_id, void (*callback)(), unsigned long interval);
        static void change_process(size_t process_id, void (*callback)());
        static void change_process(size_t process_id, unsigned long interval);

        static void heartbeat();
        static void poll_encoders();
        static void check_motors();
        static void milestone_1();

    private:
        static struct process* processes[PROCESS_COUNT];
};

extern Processes processes;


#endif //FIRMWARE_PROCESSES_H
