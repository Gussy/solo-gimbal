/*
 * led.h
 *
 *  Created on: 01/03/2015
 *      Author: angus
 */

#ifndef LED_H_
#define LED_H_

#include "PeripheralHeaderIncludes.h"

// Configure the period for each timer
#define LED_EPWM_TIMER_TBPRD	2000
#define LED_EPWM_MAX_CMP		(LED_EPWM_TIMER_TBPRD - 10)
#define LED_EPWM_MIN_CMP		10

// To keep track of which way the compare value is moving
#define LED_EPWM_CMP_UP   1
#define LED_EPWM_CMP_DOWN 0

typedef struct {
	volatile struct EPWM_REGS *EPwmRegHandle;
	Uint16 EPwm_CMPA_Direction;
	Uint16 EPwm_CMPB_Direction;
	Uint16 EPwmTimerIntCount;
	Uint16 EPwmMaxCMPA;
	Uint16 EPwmMinCMPA;
	Uint16 EPwmMaxCMPB;
	Uint16 EPwmMinCMPB;
} LED_EPWM_INFO;

void init_led(void);

interrupt void led_epwm5_isr(void);
interrupt void led_epwm6_isr(void);

#endif /* LED_H_ */
