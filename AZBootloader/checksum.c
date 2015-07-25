#include "boot/Boot.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "checksum.h"
#include "can_bootloader.h"
#include "data.h"
#include "can.h"

unsigned int location = 0;

void reset_datapointer(void) {
	location = 0;
}

Uint16 read_Data_and_Send()
{
	Uint16 retval = 0;
	retval = DATA[location++];
	retval = ((retval & 0xFF00)>>8)|((retval & 0x00FF)<<8);
	CAN_SendWordData(retval); // This will block until the send is successful
	return retval;
}

Uint16 read_Data()
{
	Uint16 retval = 0;
	retval = DATA[location++];
	retval = ((retval & 0xFF00)>>8)|((retval & 0x00FF)<<8);
	return retval;
}

Uint32 crc32_add(Uint16 value, Uint32 crc)
{
	Uint32 retval;
	retval = (crc ^ value)&0xFFFF;

	return retval;
}

int verify_data_checksum(void)
{
   Uint32 stored_checksum = 0;
   Uint32 calculated_checksum = 0xFFFFFFFF;
   reset_datapointer();

   // Asign GetWordData to the CAN-A version of the
   // function.  GetWordData is a pointer to a function.
   GetWordData = read_Data;
   if (GetWordData() != BOOTLOADER_KEY_VALUE_8BIT) return 0;
   calculated_checksum = crc32_add(BOOTLOADER_KEY_VALUE_8BIT, calculated_checksum);

   Uint16 i;
   // Read and discard the 8 reserved words.
   for(i = 1; i <= 8; i++) {
	   calculated_checksum = crc32_add(GetWordData(), calculated_checksum);
   }

   // Entry Addr
   calculated_checksum = crc32_add(GetWordData(), calculated_checksum);
   calculated_checksum = crc32_add(GetWordData(), calculated_checksum);

   struct HEADER {
      Uint16 BlockSize;
      Uint32 DestAddr;
    } BlockHeader;

    Uint16 wordData;

    // Get the size in words of the first block
    BlockHeader.BlockSize = (*GetWordData)();
    calculated_checksum = crc32_add(BlockHeader.BlockSize,calculated_checksum);


    // While the block size is > 0 copy the data
    // to the DestAddr.  There is no error checking
    // as it is assumed the DestAddr is a valid
    // memory location

    while((BlockHeader.BlockSize != (Uint16)0x0000)&&(BlockHeader.BlockSize != 0xFFFF))
    {
       BlockHeader.DestAddr = CAN_GetLongData();
       calculated_checksum = crc32_add(BlockHeader.DestAddr>>16,calculated_checksum);
       calculated_checksum = crc32_add(BlockHeader.DestAddr&0xFFFF,calculated_checksum);
       for(i = 1; i <= BlockHeader.BlockSize; i++)
       {
          extern Uint16 endRam;
          wordData = (*GetWordData)();
          calculated_checksum = crc32_add(wordData,calculated_checksum);
       }

       // Get the size of the next block
       BlockHeader.BlockSize = (*GetWordData)();
       calculated_checksum = crc32_add(BlockHeader.BlockSize,calculated_checksum);
    }

   stored_checksum = (*GetWordData)();

   reset_datapointer();

   if (stored_checksum == calculated_checksum) return 1;
   return 0;
}
