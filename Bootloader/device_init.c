#include "boot/Boot.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "hardware/pll.h"

void DeviceInit() {
	EALLOW;
	calibrate_adc();
	EDIS;

	EALLOW;
	init_xtal();
	EDIS;

	PLLset( PLL_80MHZ_SYSTEM_CLOCK_20MHZ_XTAL);

	EALLOW;
	init_gpio();	// Setup all the pins, not just the required one. So it runs just like in the application firmware
	EDIS;
}
