#ifndef __CURRENT_CONTROLLER_H__
#define __CURRENT_CONTROLLER_H__
// returns a duty cycle from 0 to 1
#include "hardware/timing.h"
#include <stdbool.h>

struct current_controller_param_t {
    float Kp;        // Feedback proportional coefficient
    float Ki;        // Feedback integrator coefficient
    float R;         // Resistance in Ohms
    float V;         // Voltage in Volts

    float Vmin;      // Minimum V in Volts

    float V_filt_tc; // Voltage filter time constant in seconds

    float Idem;      // Current demand in Amperes
    float I;         // Measured current in Amperes

    float max_dt;    // Maximum time between updates
};

struct current_controller_state_t {
    float V_filtered;
    float integrator;
    uint32_t last_run_us;
    float output;
};

struct current_controller_t {
    struct current_controller_param_t param;
    struct current_controller_state_t state;
};

void reset_current_controller(struct current_controller_t* controller);
void run_current_controller(struct current_controller_t* controller);

#endif // __CURRENT_CONTROLLER_H__
