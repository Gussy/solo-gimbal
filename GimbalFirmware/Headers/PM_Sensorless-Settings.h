#ifndef PROJ_SETTINGS_H

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

// Define the system frequency (MHz)
#define SYSTEM_FREQUENCY 80

// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

// Define the current measurement limits of the current sense circuit (full scale current is +/- this value)
#define MAX_CURRENT 2.75

//cutoff freq and time constant of the offset calibration LPF
#define WC_CAL	100.0
#define TC_CAL	1/WC_CAL

// Define the base quantites 
#define BASE_FREQ      	200           	// Base electrical frequency (Hz)

#define CURRENT_CALIBRATION_TIME_MS 2000

#endif 
