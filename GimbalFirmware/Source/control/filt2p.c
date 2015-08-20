#include "control/filt2p.h"
#include <math.h>

#ifndef M_PI
#define M_PI (float)3.14159265358979323846
#endif

float update_filt2p(struct Filt2p_Params* filt_params, struct Filt2p_State* filt_state, float sample) {
    float ret;
    float z_0 = sample - filt_state->z_1 * filt_params->a1 - filt_state->z_2 * filt_params->a2;

    if (isnan(z_0) || isinf(z_0)) {
        z_0 = sample;
    }

    ret = z_0 * filt_params->b0 + filt_state->z_1 * filt_params->b1 + filt_state->z_2 * filt_params->b2;

    filt_state->z_2 = filt_state->z_1;
    filt_state->z_1 = z_0;

    return ret;
}

void calc_butter2p(float sample_freq, float cutoff_freq, struct Filt2p_Params* ret) {
    float fr = sample_freq/cutoff_freq;
    float ohm = tan(M_PI/fr);
    float c = 1.0f+2.0f*cos(M_PI/4.0f)*ohm + ohm*ohm;

    ret->b0 = ohm*ohm/c;
    ret->b1 = 2.0f*ret->b0;
    ret->b2 = ret->b0;
    ret->a1 = 2.0f*(ohm*ohm-1.0f)/c;
    ret->a2 = (1.0f-2.0f*cos(M_PI/4.0f)*ohm+ohm*ohm)/c;
}
