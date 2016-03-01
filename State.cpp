#include "State.h"

#define FW_DEBUG

static motor motor_0 = {
    .port           = 2,
    .power          = 0,
    .desired_power  = 0,
    .direction      = 1,
    .disp           = 0,
    .disp_delta     = 0,
    .speed          = 0.0,
    .delta_speed    = 0.0,
    .last_write     = 0
};
static motor motor_1 = {
    .port           = 5,
    .power          = 0,
    .desired_power  = 0,
    .direction      = 1,
    .disp           = 0,
    .disp_delta     = 0,
    .speed          = 0.0,
    .delta_speed    = 0.0,
    .last_write     = 0
};
static motor motor_2 = {
    .port           = 4,
    .power          = 0,
    .desired_power  = 0,
    .direction      = 1,
    .disp           = 0,
    .disp_delta     = 0,
    .speed          = 0.0,
    .delta_speed    = 0.0,
    .last_write     = 0
};

const float State::velo_coupling_mat[3][3] =
{
    {-0.5,   0.8660254,   1.0},
    {-0.5,  -0.8660254,   1.0},
    {1.0,    0.0,         1.0}
};

void State::setup()
{
    strip.begin();
    strip.show();
    
    motors[0] = &motor_0;
    motors[1] = &motor_1;
    motors[2] = &motor_2;

    kicker_state = Idle;
    grabber_state = Open;
    status_led = LOW;
    pinMode(status_led_pin, OUTPUT);
    digitalWrite(status_led_pin, status_led);
}

