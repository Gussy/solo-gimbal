/*
 * adc.c
 *
 *  Created on: Feb 6, 2015
 *      Author: abamberger
 */


#include "PeripheralHeaderIncludes.h"
#include "f2806/f2806xileg_vdc_PM.h"

#define Device_cal (void   (*)(void))0x3D7C80

void init_adc()
{
    ADC_DELAY_US(ADC_usDELAY);
    AdcRegs.ADCCTL1.all=ADC_RESET_FLAG;
    asm(" NOP ");
    asm(" NOP ");

    EALLOW;

    (*Device_cal)();                      // Auto-calibrate from TI OTP

    AdcRegs.ADCCTL1.bit.ADCBGPWD   = 1; // Power up band gap

    ADC_DELAY_US(ADC_usDELAY); // Delay before powering up rest of ADC

    AdcRegs.ADCCTL1.bit.ADCREFSEL   = 1;    /* KRK */
    AdcRegs.ADCCTL1.bit.ADCREFPWD   = 1;    /* Power up reference */
    AdcRegs.ADCCTL1.bit.ADCPWDN     = 1;    /* Power up rest of ADC */
    AdcRegs.ADCCTL1.bit.ADCENABLE   = 1;    /* Enable ADC */

    AdcRegs.ADCCTL1.bit.VREFLOCONV = 1; // Internally connect VRefLo to adc channel B5

    AdcRegs.ADCCTL2.bit.CLKDIV2EN   = 1;    /* KRK, ADC clock is CPU clock /2 */
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;  /* KRK set non overlap bit to reduce ADC errors */

    asm(" RPT#100 || NOP");

    AdcRegs.ADCCTL1.bit.INTPULSEPOS=1;
    AdcRegs.ADCCTL1.bit.TEMPCONV=1;      /* KRK use ADC5 for temp measurement */

    ADC_DELAY_US(ADC_usDELAY);

    /******* CHANNEL SELECT *******/
    // Set SOC's 0, 1, 2, 3, 4, and 5 to be high priority, guaranteeing that phase current and encoder
    // measurements always complete in preference to temperature and bus voltage measurements
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 6;

    // Dummy read for ADC first read errata
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0x1;  /* ChSelect: ADC A1-> Phase A Current Sense*/
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0x5;  /* Set SOC0 start trigger on EPWM1A, due to*/
                                            /* round-robin SOC0 converts first then SOC1*/
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 0x6;  /* Set SOC0 S/H Window to 7 ADC Clock Cycles,*/
                                            /* (6 ACQPS plus 1)*/

    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 0x1;  /* ChSelect: ADC A1-> Phase A Current Sense*/
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 0x5;  /* Set SOC1 start trigger on EPWM1A, due to*/
                                            /* round-robin SOC0 converts first then SOC1*/
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 0x6;  /* Set SOC0 S/H Window to 7 ADC Clock Cycles,*/
                                            /* (6 ACQPS plus 1)*/

    // Dummy read for ADC first read errata
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 0x9;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 0x7;
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 0x6;

    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 0x9;  /* ChSelect: ADC B1-> Phase B Current Sense*/
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 0x7;  /* Set SOC0 start trigger on EPWM2A, due to*/
                                            /* round-robin SOC0 converts first then SOC1*/
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 0x6;

    AdcRegs.ADCSOC4CTL.bit.CHSEL    = 0xD;  // ADCINB5, which is internally connected to VRefLo per configuration above
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 0x7;  // Trigger on EPWM2A
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 0x6;

    AdcRegs.ADCSOC5CTL.bit.CHSEL    = 0x8;  /* ChSelect: ADC B0-> Rotor position sensor*/
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 0x7;  /* Trigger rotor position read on EPWM2A*/
    AdcRegs.ADCSOC5CTL.bit.ACQPS    = 0x6;

    // Bus voltage measurement
    AdcRegs.ADCSOC14CTL.bit.CHSEL    = 0xC;  /* ChSelect: ADC B4-> DC Bus Voltage*/
    AdcRegs.ADCSOC14CTL.bit.TRIGSEL  = 0x0;  /* Software Trigger*/
    AdcRegs.ADCSOC14CTL.bit.ACQPS    = 0x6;

    // Temperature Measurement
    AdcRegs.ADCSOC15CTL.bit.CHSEL    = 0x5;  /* ChSelect: ADC A5-> Temp KRK*/
    AdcRegs.ADCSOC15CTL.bit.TRIGSEL  = 0x0;  /* Software Trigger*/
    AdcRegs.ADCSOC15CTL.bit.ACQPS    = 0x1C; /* long sample window for temp sensor*/

    EDIS;


/*  Set up Event Trigger with CNT_zero enable for Time-base of EPWM1 */
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;     /* Enable EPWM1 SOCA */
    EPwm1Regs.ETSEL.bit.SOCASEL = 6;    /* Enable CMPB event for EPWM1 SOCA KRK*/
                                        /* CMPB reg loaded in Main ISR module, KRK */
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;     /* Generate SOCA on the 1st event */
    EPwm1Regs.ETCLR.bit.SOCA = 1;       /* Clear SOCA flag */

/* Set up Event Trigger with CNT_zero enable for Time-base of EPWM2 */
   EPwm2Regs.ETSEL.bit.SOCAEN = 1;     /* Enable EPWM2 SOCA */
   EPwm2Regs.ETSEL.bit.SOCASEL = 6;    /* Enable CMPB event for EPWM2 SOCA KRK */
                                       /* CMPB reg loaded in Main ISR module, KRK */
   EPwm2Regs.ETPS.bit.SOCAPRD = 1;     /* Generate SOCA on the 1st event */
   EPwm2Regs.ETCLR.bit.SOCA = 1;       /* Clear SOCA flag */
}
