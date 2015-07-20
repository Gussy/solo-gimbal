#include "control/filt2p.h"
#include <math.h>

float update_filt2p(struct Filt2p_Params* filt_params, struct Filt2p_State* filt_state, float sample) {
    float z_0 = sample - filt_state->z_1 * filt_params->a1 - filt_state->z_2 * filt_params->a2;

    if (isnan(z_0) || isinf(z_0)) {
        z_0 = sample;
    }

    float ret = z_0 * filt_params->b0 + filt_state->z_1 * filt_params->b1 + filt_state->z_2 * filt_params->b2;

    filt_state->z_2 = filt_state->z_1;
    filt_state->z_1 = z_0;

    return ret;
}
