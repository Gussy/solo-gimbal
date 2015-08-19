#ifndef PROJ_SETTINGS_H

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

// Define the system frequency
#define SYSTEM_FREQUENCY_HZ 80000000U

// Define the commutation loop frequency
#define COMMUTATION_FREQUENCY_HZ 8000U
#define COMMUTATION_PERIOD_SEC (1.0/COMMUTATION_FREQUENCY_HZ)

// Define the current measurement limits of the current sense circuit (full scale current is +/- this value)
#define MAX_CURRENT 2.75

//cutoff freq and time constant of the offset calibration LPF
#define WC_CAL	100.0
#define TC_CAL	1/WC_CAL

// Define the base quantites 
#define BASE_FREQ      	200           	// Base electrical frequency (Hz)

#define CURRENT_CALIBRATION_TIME_MS 2000

#endif 
