
// External functions
extern void CopyData(void);
extern Uint32 GetLongData(void);
extern void ReadReservedFn(void);


#pragma    DATA_SECTION(endRam,".endmem");
Uint16 endRam;


Uint32 CAN_Boot()
{
   Uint32 EntryAddr;

   location = 0;

   // If the missing clock detect bit is set, just
   // loop here.
   if(SysCtrlRegs.PLLSTS.bit.MCLKSTS == 1) {
      for(;;);
   }

   // Asign GetWordData to the CAN-A version of the
   // function.  GetWordData is a pointer to a function.
   GetWordData = read_Data_and_Send;

   CAN_Init();

   // If the KeyValue was invalid, abort the load
   // and return the flash entry point.
   if(GetWordData() != BOOTLOADER_KEY_VALUE_8BIT) return FLASH_ENTRY_POINT;

   ReadReservedFn();

   EntryAddr = GetLongData();

   CopyData();

   return EntryAddr;
}
