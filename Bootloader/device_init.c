#include "boot/Boot.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"

void DeviceInit()
{
	//The Device_cal function, which copies the ADC & oscillator calibration values
	// from TI reserved OTP into the appropriate trim registers, occurs automatically
	// in the Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC and oscillators to function according
	// to specification.
		EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1; // Enable ADC peripheral clock
		(*Device_cal)();					  // Auto-calibrate from TI OTP
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0; // Return ADC clock to original state
		EDIS;

	// Switch to Internal Oscillator 1 and turn off all other clock
	// sources to minimize power consumption
		EALLOW;
		SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0;
	    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL=0;  // Clk Src = INTOSC1
		SysCtrlRegs.CLKCTL.bit.XCLKINOFF=1;     // Turn off XCLKIN
	//	SysCtrlRegs.CLKCTL.bit.XTALOSCOFF=1;    // Turn off XTALOSC
		SysCtrlRegs.CLKCTL.bit.INTOSC2OFF=1;    // Turn off INTOSC2
		SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;  //Select external crystal for osc2
		SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;  //Select osc2
	    EDIS;

	// SYSTEM CLOCK speed based on Internal OSC = 10 MHz
	// 0x10=  80    MHz		(16)
	// 0xF =  75    MHz		(15)
	// 0xE =  70    MHz		(14)
	// 0xD =  65    MHz		(13)
	// 0xC =  60	MHz		(12)
	// 0xB =  55	MHz		(11)
	// 0xA =  50	MHz		(10)
	// 0x9 =  45	MHz		(9)
	// 0x8 =  40	MHz		(8)
	// 0x7 =  35	MHz		(7)
	// 0x6 =  30	MHz		(6)
	// 0x5 =  25	MHz		(5)
	// 0x4 =  20	MHz		(4)
	// 0x3 =  15	MHz		(3)
	// 0x2 =  10	MHz		(2)

    PLLset( 0x8 );	// choose from options above

    // GPIO Config registers are EALLOW protected

    EALLOW;

    // Configure the hardware ID pins first.  This is done out of order because some of the
    // pins are configured differently depending on which board we're dealing with
    //--------------------------------------------------------------------------------------
    //  GPIO-20 - PIN FUNCTION = ID Pin 0
        GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    // 0=GPIO,  1=EQEP1A,  2=MDXA,  3=COMP1OUT
        GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;     // 1=OUTput,  0=INput
        GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;     // Enable internal pullups
    //  GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;   // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO20 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-21 - PIN FUNCTION = ID Pin 1
        GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;    // 0=GPIO,  1=EQEP1B,  2=MDRA,  3=COMP2OUT
        GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;     // 1=OUTput,  0=INput
        GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;     // Enable internal pullups
    //  GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;   // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO21 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-7 - PIN FUNCTION = User LED (Active Low)
        GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;    // 0=GPIO, 1=EPWM4B, 2=SCIRXDA, 3=ECAP2
        GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;    // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;  // uncomment if --> Set Low initially
        GpioDataRegs.GPASET.bit.GPIO7 = 1;    // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------

    EDIS;
}
