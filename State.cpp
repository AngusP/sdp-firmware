#include "State.h"

static struct motor motor_0 = {
    .port         = 2,
    .power        = 0,
    .direction    = 1,
    .disp         = 0,
    .speed        = 0.0
};
static struct motor motor_1 = {
    .port         = 5,
    .power        = 0,
    .direction    = 1,
    .disp         = 0,
    .speed        = 0.0
};
static struct motor motor_2 = {
    .port         = 4,
    .power        = 0,
    .direction    = 1,
    .disp         = 0,
    .speed        = 0.0
};

void State::setup()
{
    /*motors[motor_count] = {
        &motor_1,
        &motor_2,
        &motor_3
    };*/
    motors[0] = &motor_0;
    motors[1] = &motor_1;
    motors[2] = &motor_2;

    receiving = false;
    sending = false;
}
