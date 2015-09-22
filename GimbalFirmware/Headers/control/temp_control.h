#ifndef TEMP_CONTROL_H_
#define TEMP_CONTROL_H_

struct temp_control_param_t {
    float Cw;         // heat capacity of windings, J/K
    float Ca;         // heat capacity of assembly, J/K
    float Rwa;        // heat resistance between winding and assembly, K/W
    float Rag;        // heat resistance between assembly and "ground" (air), K/W
    float R;          // nominal resistance of windings, Ohm
    float TCRw;       // temperature coefficient of resistance of windings, Ohm/K
    float Tw_lim;     // temperature limit, K
    float Tw_lim_P;   // P term for temperature control, 1/s
};

struct temp_control_state_t {
    float Tw;         // temperature delta between windings and "ground" (air), K
    float Ta;         // temperature delta between assembly and "ground" (air), K
    float I_lim;      // current limit, Amperes
};

void temp_control_update(struct temp_control_param_t* params, struct temp_control_state_t* state, float I, float dt);

#endif // TEMP_CONTROL_H_
