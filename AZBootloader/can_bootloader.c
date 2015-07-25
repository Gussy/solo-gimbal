#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "can.h"
#include "checksum.h"

#define FLASH_ENTRY_POINT 0x3F7FF6

#pragma DATA_SECTION(endRam,".endmem");
Uint16 endRam;

static void ReadReservedFn(void);
static void CopyData(void);

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

   EntryAddr = CAN_GetLongData();

   CopyData();

   return EntryAddr;
}

// This function reads 8 reserved words in the header.
// None of these reserved words are used by the this boot loader at this time,
//  they may be used in future devices for enhancments. Loaders that use these
//  words use their own read function.
static void ReadReservedFn(void)
{
    Uint16 i;
    // Read and discard the 8 reserved words.
    for(i = 1; i <= 8; i++) {
       GetWordData();
    }
}

// This routine copies multiple blocks of data from the host to the specified RAM locations.
// There is no error checking on any of the destination addresses, it is assumed
//  all addresses and block size values are correct.
// Multiple blocks of data are copied until a block size of 00 00 is encountered.
static void CopyData(void)
{

   struct HEADER {
     Uint16 BlockSize;
     Uint32 DestAddr;
   } BlockHeader;

   Uint16 wordData;
   Uint16 i;

   // Get the size in words of the first block
   BlockHeader.BlockSize = (*GetWordData)();

   // While the block size is > 0 copy the data
   // to the DestAddr.  There is no error checking
   // as it is assumed the DestAddr is a valid
   // memory location

   while(BlockHeader.BlockSize != (Uint16)0x0000)
   {
      BlockHeader.DestAddr = CAN_GetLongData();
      for(i = 1; i <= BlockHeader.BlockSize; i++)
      {
          extern Uint16 endRam;
          wordData = (*GetWordData)();

          // don't overwrite the memory space I am using.
          if ((Uint16 *)BlockHeader.DestAddr >= &endRam) {
              *(Uint16 *)BlockHeader.DestAddr++ = wordData;
          } else {
              *(Uint16 *)BlockHeader.DestAddr++;
          }
      }

      // Get the size of the next block
      BlockHeader.BlockSize = (*GetWordData)();
   }
}
