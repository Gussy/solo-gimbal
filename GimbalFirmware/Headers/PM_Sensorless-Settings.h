/* =================================================================================
File name:  PM_Sensorless-Settings.H                     
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Incremental Build Level control file.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2010	Version 1.1
=================================================================================  */
#ifndef PROJ_SETTINGS_H

//#define ENABLE_BALANCE_PROCEDURE
//#define ENABLE_CURRENT_TOGGLE
//#define ENABLE_RATE_LOOP_TUNING
//#define STANDALONE_MODE
#define USE_AVERAGE_POWER_FILTER
#define ENABLE_AXIS_CALIBRATION_PROCEDURE
#define POSITION_LOOP_TYPE 3 // 0 = basic position loop, 1 = advanced position loop, 2 = simple single pid, 3 = EKF position loop closed on copter (no gimbal position loop)

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)
#if (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#elif (DSP280x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 100
#elif (F2806x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 80
#endif

//Define system Math Type
// Select Floating Math Type for 2806x
// Select IQ Math Type for 2803x 
#if (DSP2803x_DEVICE_H==1)
#define MATH_TYPE 0 
#elif (F2806x_DEVICE_H==1)
#define MATH_TYPE 1
#endif


// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

// Define the current measurement limits of the current sense circuit (full scale current is +/- this value)
#define MAX_CURRENT 2.75

//cutoff freq and time constant of the offset calibration LPF
#define WC_CAL	100.0
#define TC_CAL	1/WC_CAL

// This machine parameters are based on 24V PM motors inside Multi-Axis +PFC package
// Define the PMSM motor parameters
//#define RS 		0.79               		// Stator resistance (ohm)
//#define RS 		3.89               		// Stator resistance (ohm) for HT2300
#define RR   	0               		// Rotor resistance (ohm) 
//#define LS   	0.0012     				  // Stator inductance (H)
#define LS   	0.00132    				// Stator inductance (H) for HT2300
//#define LR   	0						  // Rotor inductance (H)
#define LR   	0		  			    // Rotor inductance (H) for HT2300
#define LM   	0						// Magnetizing inductance (H)
#define POLES   16						// Number of poles
//#define POLES   12						// Number of poles for HT2300

// Define the base quantites 
#define BASE_VOLTAGE    38.29		    // Base peak phase voltage (volt), maximum measurable DC Bus(66.32V)/sqrt(3) 
#define BASE_CURRENT    8.6            	// Base peak phase current (amp) , maximum measurable peak current
#define BASE_FREQ      	200           	// Base electrical frequency (Hz)

#define CURRENT_CALIBRATION_TIME_MS 2000

#define THETA_MAX_MIN_HISTORY_SIZE 100

#define POSITION_LOOP_DEADBAND_COUNTS 10

#endif 
