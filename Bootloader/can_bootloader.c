#include "boot/Boot.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "can.h"

// External functions
extern void CopyData(void);
extern Uint32 GetLongData(void);
extern void ReadReservedFn(void);
extern Uint16 CAN_GetWordData();

#pragma   DATA_SECTION(endRam,".endmem");
Uint16 endRam;

extern LED_RGBA rgba_amber;
int location = 0;

Uint32 CAN_Boot()
{
   if(GetBoardHWID() == EL) {
	   init_led_periph();
	   init_led_interrupts();
	   init_led();
	   led_set_mode(LED_MODE_OFF, rgba_amber, 0);
   }

   Uint32 EntryAddr;

   location = 0;

   // If the missing clock detect bit is set, just
   // loop here.
   if(SysCtrlRegs.PLLSTS.bit.MCLKSTS == 1)
   {
      for(;;);
   }

   // Asign GetWordData to the CAN-A version of the
   // function.  GetWordData is a pointer to a function.
   GetWordData = CAN_GetWordData;

   CAN_Init();

   // If the KeyValue was invalid, abort the load
   // and return the flash entry point.
   if (GetWordData() != 0x08AA) return FLASH_ENTRY_POINT;

   ReadReservedFn();

   EntryAddr = GetLongData();

   CopyData();

   return EntryAddr;
}
