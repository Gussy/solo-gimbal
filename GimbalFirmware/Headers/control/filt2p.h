#ifndef FILT2P_H_
#define FILT2P_H_

struct Filt2p_Params {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
};

struct Filt2p_State {
    float z_1;
    float z_2;
};

float update_filt2p(struct Filt2p_Params* filt_params, struct Filt2p_State* filt_state, float sample);

void calc_butter2p(float sample_freq, float cutoff_freq, struct Filt2p_Params* ret);

#endif
