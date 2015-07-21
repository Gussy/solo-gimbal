#include "boot/Boot.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "can.h"
#include "checksum.h"

// External functions
extern void CopyData(void);
extern Uint32 GetLongData(void);
extern void ReadReservedFn(void);
extern Uint16 CAN_GetWordData();

#pragma DATA_SECTION(endRam,".endmem");
Uint16 endRam;

Uint32 CAN_Boot(GimbalAxis axis)
{
   Uint32 EntryAddr;

   if(axis == AZ) reset_datapointer();

   // If the missing clock detect bit is set, just
   // loop here.
   if(SysCtrlRegs.PLLSTS.bit.MCLKSTS == 1) {
      for(;;);
   }

   // Asign GetWordData to the CAN-A version of the
   // function.  GetWordData is a pointer to a function.
   if(axis == AZ) {
      GetWordData = read_Data_and_Send;
   } else {
      GetWordData = CAN_GetWordData;
   }

   CAN_Init(axis);

   // If the KeyValue was invalid, abort the load
   // and return the flash entry point.
   if(GetWordData() != BOOTLOADER_KEY_VALUE_8BIT) return FLASH_ENTRY_POINT;

   ReadReservedFn();

   EntryAddr = GetLongData();

   CopyData();

   return EntryAddr;
}
