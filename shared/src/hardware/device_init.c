#include "hardware/device_init.h"
#include "PM_Sensorless.h"
#include "hardware/HWSpecific.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/led.h"
#include "hardware/pll.h"
#include "hardware/watchdog.h"
#include "hardware/interrupts.h"

#define Device_cal (void   (*)(void))0x3D7C80

static void calibrate_adc();
static void init_xtal();
static void init_peripheral_clocks();
static void init_gpio();

void DeviceInit(void)
{
	WatchDogDisable();	// Disable the watchdog initially
	DINT;			// Global Disable all Interrupts
	IER = 0x0000;	// Disable CPU interrupts
	IFR = 0x0000;	// Clear all CPU interrupt flags

	EALLOW;
	calibrate_adc();
	EDIS;

	EALLOW;
	init_xtal();
    EDIS;

	PLLset( PLL_80MHZ_SYSTEM_CLOCK_20MHZ_XTAL );

	// Initialise interrupt controller and Vector Table
	// to defaults for now. Application ISR mapping done later.
	PieCntlInit();		
	PieVectTableInit();

   EALLOW; // below registers are "protected", allow access.
   init_peripheral_clocks();
   init_gpio();
   EDIS;	// Disable register access
}

static void calibrate_adc(){
	//The Device_cal function, which copies the ADC & oscillator calibration values
	// from TI reserved OTP into the appropriate trim registers, occurs automatically
	// in the Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC and oscillators to function according
	// to specification.
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1; // Enable ADC peripheral clock
	(*Device_cal)();					  // Auto-calibrate from TI OTP
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0; // Return ADC clock to original state
}
static void init_xtal(){
	SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0;
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL=0;  // Clk Src = INTOSC1
	SysCtrlRegs.CLKCTL.bit.XCLKINOFF=1;     // Turn off XCLKIN
//	SysCtrlRegs.CLKCTL.bit.XTALOSCOFF=1;    // Turn off XTALOSC
	SysCtrlRegs.CLKCTL.bit.INTOSC2OFF=1;    // Turn off INTOSC2
	SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;  //Select external crystal for osc2
	SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;  //Select osc2
}

static void init_peripheral_clocks(){
	// HIGH / LOW SPEED CLOCKS prescale register settings
	   SysCtrlRegs.LOSPCP.all = 0x0002;		// Sysclk / 4 (20 MHz)
	   SysCtrlRegs.XCLK.bit.XCLKOUTDIV=0;	//divide by 4 default

	// PERIPHERAL CLOCK ENABLES
	//---------------------------------------------------
	// If you are not using a peripheral you may want to switch
	// the clock off to save power, i.e. set to =0
	//
	// Note: not all peripherals are available on all 280x derivates.
	// Refer to the datasheet for your particular device.

	   // Need ADC for reading phase currents, rotor position from analog pot, die temp
	   SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;    // ADC
	   //------------------------------------------------
	   // I2C is used for GoPro HeroBus communication
	   SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;   // I2C
	   //------------------------------------------------
	   // SPI-A reads from the gyro
	   // SPI-B is for debug use, and may be disabled for production
	   SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;     // SPI-A
	   SysCtrlRegs.PCLKCR0.bit.SPIBENCLK = 1;	// SPI-B
	   //------------------------------------------------
	   SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 0;	// McBSP-A
	   //------------------------------------------------
	   // SCI-B is used for MAVLink communication with the parent system
	   SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 0;   // SCI-A
	   SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 1;	// SCI-B
	   //------------------------------------------------
	   // CAN-A is used for communication with the other gimbal axes
	   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK = 1;  // eCAN-A
	   //------------------------------------------------
	   // PWM modules 1-3 are used for motor commutation and triggering ADC reads
	   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1
	   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
	   SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // ePWM3
	   SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0;  // ePWM4
	   SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 0;	// ePWM5
	   SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 0;	// ePWM6
	   SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 0;	// ePWM7
	   SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 0;	// ePWM8
	   //------------------------------------------------
	   SysCtrlRegs.PCLKCR0.bit.HRPWMENCLK = 0;	// HRPWM
	   //------------------------------------------------
	   // Enable globally synchronized PWM clocks
	   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Enable TBCLK
	   //------------------------------------------------
	   // ECAP-1 used as a timer for the main commutation loop
	   SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 1;  // eCAP1
	   SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK = 0;  // eCAP2
	   SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK = 0;  // eCAP3
	   //------------------------------------------------
	   SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 0;  // eQEP1
	   SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK = 0;  // eQEP2
	   //------------------------------------------------
	   SysCtrlRegs.PCLKCR3.bit.COMP1ENCLK = 0;	// COMP1
	   SysCtrlRegs.PCLKCR3.bit.COMP2ENCLK = 0;	// COMP2
	   SysCtrlRegs.PCLKCR3.bit.COMP3ENCLK = 0;  // COMP3
	   //------------------------------------------------
	   // CPU timers used for low speed (1ms, 5ms, 50ms) tasks
	   SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1;	// CPUTIMER0
	   SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = 1;	// CPUTIMER1
	   SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 1;  // CPUTIMER2
	   //------------------------------------------------
	   SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 0;	//SYSCLKOUT on GPIO
	   //------------------------------------------------
	   SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 0;	// DMA
	   //------------------------------------------------
	   SysCtrlRegs.PCLKCR3.bit.CLA1ENCLK = 0;	// CLA
	   //------------------------------------------------
}

static void init_gpio(){
//--------------------------------------------------------------------------------------
// GPIO (GENERAL PURPOSE I/O) CONFIG
//--------------------------------------------------------------------------------------
//-----------------------
// QUICK NOTES on USAGE:
//-----------------------
// If GpioCtrlRegs.GP?MUX?bit.GPIO?= 1, 2 or 3 (i.e. Non GPIO func), then leave
//	rest of lines commented
// If GpioCtrlRegs.GP?MUX?bit.GPIO?= 0 (i.e. GPIO func), then:
//	1) uncomment GpioCtrlRegs.GP?DIR.bit.GPIO? = ? and choose pin to be IN or OUT
//	2) If IN, can leave next to lines commented
//	3) If OUT, uncomment line with ..GPACLEAR.. to force pin LOW or
//			   uncomment line with ..GPASET.. to force pin HIGH or

// Configure the hardware ID pins first.  This is done out of order because some of the
// pins are configured differently depending on which board we're dealing with
//--------------------------------------------------------------------------------------
//  GPIO-20 - PIN FUNCTION = ID Pin 0
   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    // 0=GPIO,  1=EQEP1A,  2=MDXA,  3=COMP1OUT
   GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;     // 1=OUTput,  0=INput
   GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;                // Enable internal pullups
//  GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;   // uncomment if --> Set Low initially
//  GpioDataRegs.GPASET.bit.GPIO20 = 1;     // uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-21 - PIN FUNCTION = ID Pin 1
   GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;    // 0=GPIO,  1=EQEP1B,  2=MDRA,  3=COMP2OUT
   GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;     // 1=OUTput,  0=INput
   GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;                // Enable internal pullups
//  GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;   // uncomment if --> Set Low initially
//  GpioDataRegs.GPASET.bit.GPIO21 = 1;     // uncomment if --> Set High initially
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
//  GPIO-00 - PIN FUNCTION = PWM_H/Ln_PHA FOR 3DR MDCB
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;		// 0=GPIO, 1=EPWM1A, 2=Resv, 3=Resv
//	GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO0 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-01 - PIN FUNCTION = PWM_OE_PHA for 3DR MDCB
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;		// 0=GPIO, 1=EPWM1B, 2=EMU (0), 3=COMP1OUT
	GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO1 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-02 - PIN FUNCTION = PWM_H/Ln_PHB for 3DR MDCB
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;		// 0=GPIO, 1=EPWM2A, 2=Resv, 3=Resv
//	GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO2 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-03 - PIN FUNCTION = PWM_OE_PHB for 3DR MDCB
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;		// 0=GPIO, 1=EPWM2B, 2=SPISOMIA, 3=COMP2OUT
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO3 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-04 - PIN FUNCTION = CH1_PWM_H/Ln_PHC for 3DR MDCB
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;		// 0=GPIO, 1=EPWM3A, 2=Resv, 3=Resv
//	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO4 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-05 - PIN FUNCTION = PWM_OE_PHC for 3DR MDCB
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;		// 0=GPIO, 1=EPWM3B, 2=SPISIMOA, 3=ECAP1
	GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO5 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-06 - PIN FUNCTION = GoPro on flag
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;		// 0=GPIO, 1=EPWM4A, 2=EPWMSYNCI, 3=EPWMSYNCO
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 0;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO6 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-07 - PIN FUNCTION = User LED (on-board debug LED), Active Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;		// 0=GPIO, 1=EPWM4B, 2=SCIRXDA, 3=ECAP2
	GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO7 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-08 - PIN FUNCTION = Gimbal Status LED Red
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;		// 0=GPIO,  1=EPWM5A,  2=Resv,  3=ADCSOCA
	GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;		// 1=OUTput,  0=INput
	GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;      // Disable internal pullup
	GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO8 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-09 - PIN FUNCTION = Gimbal Status LED Green
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;		// 0=GPIO,  1=EPWM5B,  2=SCITXDB,  3=ECAP3
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;		// 1=OUTput,  0=INput
	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;      // Disable internal pullup
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO9 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-10 - PIN FUNCTION = Gimbal Status LED Blue
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;	// 0=GPIO,  1=EPWM6A,  2=Resv,  3=ADCSOCB
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;		// 1=OUTput,  0=INput
	GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;      // Disable internal pullup
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO10 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-11 - PIN FUNCTION = HeroBus 5v Charging Overcurrent/Overtemp flag (Active Low)
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;	// 0=GPIO,  1=EPWM6B,  2=SCIRXDB,  3=ECAP1
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO11 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-12 - PIN FUNCTION = Gyro Interrupt Line, Active Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;	// 0=GPIO, 1=TZ1n, 2=SCITXDA, 3=SPISIMOB
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;		// 1=OUTput,  0=INput
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;     // Disable internal pullup (Gyro Configured for Push-Pull on interrupt line)
//	GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO12 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-13 - PIN FUNCTION = PWM Fault Interrupt Line, Active Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;	// 0=GPIO,  1=TZ2,  2=Resv,  3=SPISOMIB
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;		// 1=OUTput,  0=INput
	GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;     // Disable internal pullup, this signal is externally pulled up
//	GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO13 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-14 - PIN FUNCTION = Debug SPI port Clk
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3;	// 0=GPIO,  1=TZ3,  2=SCITXDB,  3=SPICLKB
//	GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO14 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-15 - PIN FUNCTION = Debug SPI port CS, Active Low (used as a GPIO by the SPI code)
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;	// 0=GPIO,  1=ECAP2,  2=SCIRXDB,  3=SPISTEB
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO15 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
	switch (GetBoardHWID()) {
	case AZ:
	    // If this is the AZ board, GPIOs 16, 17, 18 and 19 get configured as SCI pins
	    //--------------------------------------------------------------------------------------
        //  GPIO-16 - PIN FUNCTION = RTS in from copter
            GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;    // 0=GPIO, 1=SPISIMOA, 2=Resv CAN-B, 3=TZ2n
            GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;     // 1=OUTput,  0=INput
        //  GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;   // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO16 = 1;     // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //  GPIO-17 - PIN FUNCTION = CTS out to copter
            GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;    // 0=GPIO, 1=SPISOMIA, 2=Resv CAN-B, 3=TZ3n
            GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;     // 1=OUTput,  0=INput
            GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;   // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO17 = 1;     // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------
        //  GPIO-18 - PIN FUNCTION = Tx to copter
            GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;    // 0=GPIO, 1=SPICLKA, 2=SCITXDB, 3=XCLKOUT
        //  GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;     // 1=OUTput,  0=INput
        //  GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;   // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO18 = 1;     // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //  GPIO-19 - PIN FUNCTION = Rx from copter
            GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;    // 0=GPIO, 1=SPISTEA, 2=SCIRXDB, 3=ECAP1
        //  GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     // 1=OUTput,  0=INput
        //  GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;   // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO19 = 1;     // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
	    break;

	case EL:
		// Initialise the Beacon LED specific peripherals
		init_led_periph();
	case ROLL:
	    // On the EL and ROLL boards, GPIOs 16, 17, 18 and 19 get configured as SPI pins
	    //--------------------------------------------------------------------------------------
	    //  GPIO-16 - PIN FUNCTION = Gyro SPI port MOSI
	        GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;    // 0=GPIO, 1=SPISIMOA, 2=Resv CAN-B, 3=TZ2n
	    //  GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;     // 1=OUTput,  0=INput
	    //  GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;   // uncomment if --> Set Low initially
	    //  GpioDataRegs.GPASET.bit.GPIO16 = 1;     // uncomment if --> Set High initially
	    //--------------------------------------------------------------------------------------
	    //  GPIO-17 - PIN FUNCTION = Gyro SPI port MISO
	        GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;    // 0=GPIO, 1=SPISOMIA, 2=Resv CAN-B, 3=TZ3n
	    //  GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;     // 1=OUTput,  0=INput
	    //  GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;   // uncomment if --> Set Low initially
	    //  GpioDataRegs.GPASET.bit.GPIO17 = 1;     // uncomment if --> Set High initially
	    //--------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------
        //  GPIO-18 - PIN FUNCTION = Gyro SPI port Clk
            GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;    // 0=GPIO, 1=SPICLKA, 2=SCITXDB, 3=XCLKOUT
        //  GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;     // 1=OUTput,  0=INput
        //  GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;   // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO18 = 1;     // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //  GPIO-19 - PIN FUNCTION = Gyro SPI port CS, Active Low (used as a GPIO by the SPI code)
            GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    // 0=GPIO, 1=SPISTEA, 2=SCIRXDB, 3=ECAP1
            GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     // 1=OUTput,  0=INput
        //  GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;   // uncomment if --> Set Low initially
            GpioDataRegs.GPASET.bit.GPIO19 = 1;     // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
	    break;
	}
//--------------------------------------------------------------------------------------
//  GPIO-22 - PIN FUNCTION = Power on control to camera, Active Low
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;	// 0=GPIO,  1=EQEP1S,  2=MCLKXA,  3=SCITXDB
	// Set as an input initially, set the initial state, and then set as an output.
	// This is because the act of configuring the GPIO while it's already an output is
	// enough to turn the camera on, so we want to make sure the output is configured high
	// before it's configured as an output
	GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;		// 1=OUTput,  0=INput
	GpioDataRegs.GPASET.bit.GPIO22 = 1;		// uncomment if --> Set High initially
	GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;
//--------------------------------------------------------------------------------------
//  GPIO-23 - PIN FUNCTION = HeroBus 5V power enable, Active low
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;	// 0=GPIO,  1=EQEP1I,  2=MFSXA,  3=SCIRXDB
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1;     // Disable the internal pullup on this pin
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO23 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-24 - PIN FUNCTION = Debug SPI port MOSI
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;	// 0=GPIO,  1=ECAP1,  2=EQEP2A,  3=SPISIMOB
//	GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO24 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-25 - PIN FUNCTION = Debug SPI port MISO
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;	// 0=GPIO,  1=ECAP2,  2=EQEP2B,  3=SPISOMIB
//	GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO25 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-26 - PIN FUNCTION = GoPro I2C request line, Active Low
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;	// 0=GPIO,  1=ECAP3,  2=EQEP2I,  3=SPICLKB
	// Set as an input initially, set the initial state, and then set as an output.
	// This is to make sure we don't accidentally output any active low glitches to the
	// camera while we're configuring the GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;		// 1=OUTput,  0=INput
	GpioDataRegs.GPASET.bit.GPIO26 = 1;		// uncomment if --> Set High initially
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;
//--------------------------------------------------------------------------------------
//  GPIO-27 - PIN FUNCTION = PWM Reset, Active Low
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;	// 0=GPIO,  1=HRCAP2,  2=EQEP2S,  3=SPISTEB
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO27 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-28 - PIN FUNCTION = UART RX, non-isolated NOTE: Temporary GPIO for debugging
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;	// 0=GPIO,  1=SCIRXDA,  2=SDAA,  3=TZ2
	GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO28 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-29 - PIN FUNCTION = UART TX, non-isolated NOTE: Temporary GPIO for debugging
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;	// 0=GPIO,  1=SCITXDA,  2=SCLA,  3=TZ3
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO29 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-30 - PIN FUNCTION = CAN RX
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;	// 0=GPIO,  1=CANRXA,  2=EQEP2I,  3=EPWM7A
//	GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO30 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-31 - PIN FUNCTION = CAN TX
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;	// 0=GPIO,  1=CANTXA,  2=EQEP2S,  3=EPWM8A
//	GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO31 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//  GPIO-32 - PIN FUNCTION = GoPro I2C SDA
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;	// 0=GPIO,  1=SDAA,  2=EPWMSYNCI,  3=ADCSOCA
//	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPBSET.bit.GPIO32 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-33 - PIN FUNCTION = GoPro I2C SCL
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;	// 0=GPIO,  1=SCLA,  2=EPWMSYNCO,  3=ADCSOCB
//	GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPBSET.bit.GPIO33 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-34 - PIN FUNCTION = Tied to +3.3v
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;	// 0=GPIO,  1=COMP2OUT,  2=Resv,  3=COMP3OUT
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPBSET.bit.GPIO34 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
// GPIO 35-38 are defaulted to JTAG usage, and are not shown here to enforce JTAG debug
// usage. 
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//  GPIO-39 - PIN FUNCTION = PWM Enable
	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;	// 0=GPIO,  1=Resv,  2=Resv,  3=Resv
	GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPBSET.bit.GPIO39 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
}
