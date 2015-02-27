/*
 * led.c
 *
 *  Created on: 01/03/2015
 *      Author: angus
 */

#include "F2806x_EPwm_defines.h"
#include "hardware/device_init.h"
#include "hardware/led.h"

static void update_compare(LED_EPWM_INFO*);

LED_EPWM_INFO epwm5_info;
LED_EPWM_INFO epwm6_info;

void init_led()
{
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

	// Setup TBCLK
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;			// Count up
	EPwm5Regs.TBPRD = LED_EPWM_TIMER_TBPRD;				// Set timer period
	EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Disable phase loading
	EPwm5Regs.TBPHS.half.TBPHS = 0x0000;				// Phase is 0
	EPwm5Regs.TBCTR = 0x0000;							// Clear counter
	EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;			// Clock ratio to SYSCLKOUT
	EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV2;

	// Setup shadow register load on ZERO
	EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set Compare values
	EPwm5Regs.CMPA.half.CMPA = LED_EPWM_MIN_CMP;		// Set compare A value
	EPwm5Regs.CMPB = LED_EPWM_MAX_CMP;					// Set Compare B value

	// Set actions
	EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;					// Set PWM1A on Zero
	EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;				// Clear PWM1A on event A, up count
	EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;					// Set PWM1B on Zero
	EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;				// Clear PWM1B on event B, up count

	// Interrupt where we will change the Compare Values
	EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;			// Select INT on Zero event
	EPwm5Regs.ETSEL.bit.INTEN = 1;						// Enable INT
	EPwm5Regs.ETPS.bit.INTPRD = ET_3RD;					// Generate INT on 3rd event

	// Information used to keep track of the direction the CMPA/CMPB values are
	// moving, the min and max allowed values and a pointer to the correct ePWM registers
	epwm5_info.EPwm_CMPA_Direction = LED_EPWM_CMP_UP;
	epwm5_info.EPwm_CMPB_Direction = LED_EPWM_CMP_DOWN;
	epwm5_info.EPwmTimerIntCount = 0;					// Zero the interrupt counter
	epwm5_info.EPwmRegHandle = &EPwm5Regs;				// Set the pointer to the ePWM module
	epwm5_info.EPwmMaxCMPA = LED_EPWM_MAX_CMP;			// Setup min/max CMPA/CMPB values
	epwm5_info.EPwmMinCMPA = LED_EPWM_MIN_CMP;
	epwm5_info.EPwmMaxCMPB = LED_EPWM_MAX_CMP;
	epwm5_info.EPwmMinCMPB = LED_EPWM_MIN_CMP;

	// Setup TBCLK
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;			// Count up
	EPwm6Regs.TBPRD = LED_EPWM_TIMER_TBPRD*2;			// Set timer period
	EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;				// Disable phase loading
	EPwm6Regs.TBPHS.half.TBPHS = 0x0000;				// Phase is 0
	EPwm6Regs.TBCTR = 0x0000;							// Clear counter
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;			// Clock ratio to SYSCLKOUT
	EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV2;

	// Setup shadow register load on ZERO
	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set Compare values
	EPwm6Regs.CMPA.half.CMPA = LED_EPWM_MIN_CMP;		// Set compare A value

	// Set actions
	EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;					// Set PWM1A on Zero
	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;				// Clear PWM1A on event A, up count

	// Interrupt where we will change the Compare Values
	EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;			// Select INT on Zero event
	EPwm6Regs.ETSEL.bit.INTEN = 1;						// Enable INT
	EPwm6Regs.ETPS.bit.INTPRD = ET_3RD;					// Generate INT on 3rd event

	// Information used to keep track of the direction the CMPA/CMPB values are
	// moving, the min and max allowed values and a pointer to the correct ePWM registers
	epwm6_info.EPwm_CMPA_Direction = LED_EPWM_CMP_UP;
	epwm6_info.EPwmTimerIntCount = 0;					// Zero the interrupt counter
	epwm6_info.EPwmRegHandle = &EPwm6Regs;				// Set the pointer to the ePWM module
	epwm6_info.EPwmMaxCMPA = LED_EPWM_MAX_CMP*2;		// Setup min/max CMPA/CMPB values
	epwm6_info.EPwmMinCMPA = LED_EPWM_MIN_CMP;

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;
}

interrupt void led_epwm5_isr(void)
{
	// Update the CMPA and CMPB values
	update_compare(&epwm5_info);

	// Clear INT flag for this timer
	EPwm5Regs.ETCLR.bit.INT = 1;

	// Acknowledge this interrupt to receive more interrupts from group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void led_epwm6_isr(void)
{
	// Update the CMPA and CMPB values
	update_compare(&epwm6_info);

	// Clear INT flag for this timer
	EPwm6Regs.ETCLR.bit.INT = 1;

	// Acknowledge this interrupt to receive more interrupts from group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

static void update_compare(LED_EPWM_INFO *epwm_info)
{
	// Every interrupt, change the CMPA/CMPB values
	if(epwm_info->EPwmTimerIntCount == 1)
	{
		epwm_info->EPwmTimerIntCount = 0;

		// If we were increasing CMPA, check to see if
		// we reached the max value.  If not, increase CMPA
		// else, change directions and decrease CMPA
		if(epwm_info->EPwm_CMPA_Direction == LED_EPWM_CMP_UP)
		{
			if(epwm_info->EPwmRegHandle->CMPA.half.CMPA < epwm_info->EPwmMaxCMPA)
			{
				epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
			}
			else
			{
				epwm_info->EPwm_CMPA_Direction = LED_EPWM_CMP_DOWN;
				epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
			}
		}

		// If we were decreasing CMPA, check to see if
		// we reached the min value.  If not, decrease CMPA
		// else, change directions and increase CMPA
		else
		{
			if(epwm_info->EPwmRegHandle->CMPA.half.CMPA == epwm_info->EPwmMinCMPA)
			{
				epwm_info->EPwm_CMPA_Direction = LED_EPWM_CMP_UP;
				epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
			}
			else
			{
				epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
			}
		}

		// If we were increasing CMPB, check to see if
		// we reached the max value.  If not, increase CMPB
		// else, change directions and decrease CMPB
		if(epwm_info->EPwm_CMPB_Direction == LED_EPWM_CMP_UP)
		{
			if(epwm_info->EPwmRegHandle->CMPB < epwm_info->EPwmMaxCMPB)
			{
				epwm_info->EPwmRegHandle->CMPB++;
			}
			else
			{
				epwm_info->EPwm_CMPB_Direction = LED_EPWM_CMP_DOWN;
				epwm_info->EPwmRegHandle->CMPB--;
			}
		}

		// If we were decreasing CMPB, check to see if
		// we reached the min value.  If not, decrease CMPB
		// else, change directions and increase CMPB

		else
		{
			if(epwm_info->EPwmRegHandle->CMPB == epwm_info->EPwmMinCMPB)
			{
				epwm_info->EPwm_CMPB_Direction = LED_EPWM_CMP_UP;
				epwm_info->EPwmRegHandle->CMPB++;
			}
			else
			{
				epwm_info->EPwmRegHandle->CMPB--;
			}
		}
	}
	else
	{
		epwm_info->EPwmTimerIntCount++;
	}

	return;
}
