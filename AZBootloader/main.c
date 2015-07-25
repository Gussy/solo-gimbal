#include "boot/Boot.h"
#include "hardware/pll.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "mavlink_bootloader.h"
#include "can_bootloader.h"
#include "checksum.h"

// Included for Flash_CPUScaleFactor and Flash_CallbackPtr
#define FLASH_F2806x 1
#include "Flash2806x_API_Library.h"

#define BORTRIM (Uint16 *)0x0986

// Empty setup functions so this code can run DeviceInit()
void PieCntlInit(void){}
void PieVectTableInit(void){}

static const LED_RGBA rgba_amber_dim = {.red=255, .green=160, .blue=0, .alpha=10};

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

	  const GimbalAxis axis = (GimbalAxis)GetBoardHWID();

	  if(axis == AZ && !verify_data_checksum()) {
          // Enter the MAVLink bootloader
          MAVLINK_Flash(); // This never returns
	  }

      // Enable the beacon LED on the elevation board only
      if(axis == EL) {
          init_led();
          led_set_mode(LED_MODE_SOLID, rgba_amber_dim, 0);
      }

      EntryAddr = CAN_Boot(axis);
      if(axis == AZ) reset_datapointer();

      return EntryAddr;
}
