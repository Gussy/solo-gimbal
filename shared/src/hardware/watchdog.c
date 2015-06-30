#include "hardware/device_init.h"

//---------------------------------------------------------------
// This module disables the watchdog timer.
//---------------------------------------------------------------

void watchdog_disable()
{
    // Disable watchdog module
	EALLOW;
	SysCtrlRegs.WDCR = 0x0068;
	EDIS;
}

//---------------------------------------------------------------
// This module enables the watchdog timer.
//---------------------------------------------------------------

void watchdog_enable()
{
	EALLOW;
	SysCtrlRegs.WDCR = 0x0028;               // Enable watchdog module
	SysCtrlRegs.WDKEY = 0x55;                // Clear the WD counter
	SysCtrlRegs.WDKEY = 0xAA;
	EDIS;
}

void watchdog_reset()
{
	// Enable watchdog
	watchdog_enable();

	// This should never be reached.
	for(;;);
}

void watchdog_immediate_reset() {
	// Enable watchdog
	watchdog_enable();

	// Cause a device reset by writing incorrect values into WDCHK
	EALLOW;
	SysCtrlRegs.WDCR = 0x0010;
	EDIS;

	// This should never be reached.
	for(;;);

}
