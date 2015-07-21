#include "boot/Boot.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "mavlink_bootloader.h"
#include "can_bootloader.h"
#include "checksum.h"

// empty setup functions so this code can run  DeviceInit()
void PieCntlInit(void){}
void PieVectTableInit(void){}

Uint32 SelectBootMode()
{
	  Uint32 EntryAddr;

	  EALLOW;

	  // Watchdog Service
	  SysCtrlRegs.WDKEY = 0x0055;
	  SysCtrlRegs.WDKEY = 0x00AA;

	  // Before waking up the flash
	  // set the POR to the minimum trip point
	  // If the device was configured by the factory
	  // this write will have no effect.

	  *BORTRIM = 0x0100;

	  // At reset we are in /4 mode.  Change to /1
	  // Calibrate the ADC and internal OSCs
	  SysCtrlRegs.PLLSTS.bit.DIVSEL = DIVSEL_BY_1;
	  SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
	  (*Device_cal)();
	  SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0;

	  // Init two locations used by the flash API with 0x0000
	  Flash_CPUScaleFactor = 0;
	  Flash_CallbackPtr = 0;
	  EDIS;

	  DeviceInit();

	  // Read the password locations - this will unlock the
	  // CSM only if the passwords are erased.  Otherwise it
	  // will not have an effect.
	  CsmPwl.PSWD0;
	  CsmPwl.PSWD1;
	  CsmPwl.PSWD2;
	  CsmPwl.PSWD3;
	  CsmPwl.PSWD4;
	  CsmPwl.PSWD5;
	  CsmPwl.PSWD6;
	  CsmPwl.PSWD7;

	  EALLOW;

	  GimbalAxis axis = (GimbalAxis)GetBoardHWID();
	  if(axis == AZ) {
          if (verify_data_checksum()) {
              // Enter the CAN bootloader (transmitter)
              EntryAddr = CAN_Boot(axis);
              reset_datapointer();
          } else {
              // Enter the MAVLink bootloader
              MAVLINK_Flash(); // This never returns
          }
	  } else {
          // Enable the beacon LED on the elevation board only
          if(axis == EL) {
              init_led();
          }

          // Enter the CAN bootloader (receiver)
          EntryAddr = CAN_Boot(axis);
	  }
	return EntryAddr;
}
