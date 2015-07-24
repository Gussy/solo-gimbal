#include "hardware/watchdog.h"
#include "PeripheralHeaderIncludes.h"

void watchdog_enable()
{
	EALLOW;
    // Enable watchdog module
	SysCtrlRegs.WDCR = 0x0028;
	EDIS;

	watchdog_service();
}

void watchdog_disable()
{
    EALLOW;
    // Disable watchdog module
    SysCtrlRegs.WDCR = 0x0068;
    EDIS;
}

void watchdog_service()
{
    EALLOW;
    // Clear the WD counter
    SysCtrlRegs.WDKEY = 0x55;
    SysCtrlRegs.WDKEY = 0xAA;
    EDIS;
}

void watchdog_device_reset()
{
	// Enable watchdog
	watchdog_enable();

	// This should never be reached.
	for(;;);
}

void watchdog_immediate_device_reset() {
	// Enable watchdog
	watchdog_enable();

	// Cause a device reset by writing incorrect values into WDCHK
	EALLOW;
	SysCtrlRegs.WDCR = 0x0010;
	EDIS;

	// This should never be reached.
	for(;;);

}
