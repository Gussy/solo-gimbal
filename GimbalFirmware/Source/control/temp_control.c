#include "control/temp_control.h"
#include <math.h>

void temp_control_update(struct temp_control_param_t* params, struct temp_control_state_t* state, float I, float dt) {
    float I2 = I*I;
    float Tw_dot = ((1.0+params->TCRw*state->Tw)*params->R*I2 - (state->Tw - state->Ta)/params->Rwa)/params->Cw;
    float Ta_dot = ((state->Tw - state->Ta)/params->Rwa - state->Ta/params->Rag)/params->Ca;
    state->Tw += dt*Tw_dot;
    state->Ta += dt*Ta_dot;

    float Tw_dot_des = (params->Tw_lim - state->Tw) * params->Tw_lim_P;
    state->I_lim = sqrt((Tw_dot_des*params->Cw + (state->Tw - state->Ta)/params->Rwa) / (params->TCRw*params->R*state->Tw + params->R));
}
