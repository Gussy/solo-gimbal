#include "control/current_controller.h"
#include "hardware/timing.h"

void run_current_controller(struct current_controller_t* controller) {
    uint32_t tnow_us = micros();
    float dt = (tnow_us-controller->state.last_run_us)*1.0e-6;
    controller->state.last_run_us = tnow_us;
    if (dt > controller->param.max_dt) {
        dt = controller->param.max_dt;
    }
    float Ierr = controller->param.Idem - controller->param.I;
    float voltage_demanded = 0;

    if (controller->param.Ki == 0.0) {
        controller->state.integrator = 0.0;
    }

    float V = controller->param.V;
    if (V < controller->param.Vmin) {
        V = controller->param.Vmin;
    }

    // apply filter to voltage
    float V_filt_alpha = dt/(dt+controller->param.V_filt_tc);
    controller->state.V_filtered += (controller->param.V - controller->state.V_filtered) * V_filt_alpha;

    // feed-forward term - V=I*R
    voltage_demanded += controller->param.R * controller->param.Idem;

    // feedback proportional term
    voltage_demanded += controller->param.Kp * Ierr;

    // add integrator state
    voltage_demanded += controller->state.integrator;

    // feedback integral term
    // determine if output is saturated, don't allow integrator to saturate us more
    if (voltage_demanded > controller->state.V_filtered) {
        if (Ierr < 0) {
            float integrator_increment = Ierr * dt * controller->param.Ki;
            controller->state.integrator += integrator_increment;
            voltage_demanded += integrator_increment;
        }
    } else if (voltage_demanded < -controller->state.V_filtered) {
        if (Ierr > 0) {
            float integrator_increment = Ierr * dt * controller->param.Ki;
            controller->state.integrator += integrator_increment;
            voltage_demanded += integrator_increment;
        }
    } else {
        float integrator_increment = Ierr * dt * controller->param.Ki;
        controller->state.integrator += integrator_increment;
        voltage_demanded += integrator_increment;
    }

    // scale to PWM duty cycle
    controller->state.output = voltage_demanded / controller->state.V_filtered;

    // output saturation
    if (controller->state.output > 1.0) {
        controller->state.output = 1.0;
    } else if (controller->state.output < -1.0) {
        controller->state.output = -1.0;
    }
}
