#ifndef AVERAGE_POWER_FILTER_H_
#define AVERAGE_POWER_FILTER_H_

typedef struct {
    float iq_filter;
    float iq_filter_prev;
    float alpha;
    float current_limit; // In Amps^2
} AveragePowerFilterParms;

void init_average_power_filter(AveragePowerFilterParms* filter_parms, int current_sample_freq, int tau, float current_limit);
unsigned char run_average_power_filter(AveragePowerFilterParms* filter_parms, float iq_current);

#endif /* AVERAGE_POWER_FILTER_H_ */
