
#include "flash/flash.h"
#include "Flash2806x_API_Library.h"
#include "F2806x_SysCtrl.h"


/*--- Callback function.  Function specified by defining Flash_CallbackPtr */
void MyCallbackFunction(void);
Uint32 MyCallbackCounter; // Just increment a counter in the callback function

/*--- Global variables used to interface to the flash routines */
FLASH_ST FlashStatus;

/*---------------------------------------------------------------------------
  Data/Program Buffer used for testing the flash API functions
---------------------------------------------------------------------------*/
#define  WORDS_IN_FLASH_BUFFER 0x100               // Programming data buffer, Words
Uint16  Buffer[WORDS_IN_FLASH_BUFFER];

typedef struct {
     Uint16 *StartAddr;
     Uint16 *EndAddr;
} SECTOR;

#define OTP_START_ADDR  0x3D7800
#define OTP_END_ADDR    0x3D7BFF


#define START_ADDR 		0x3D8000
#define FLASH_END_ADDR  0x3F7FFF

extern SECTOR Sector[8];

/*---------------------------------------------------------------------------
   These key values are used to unlock the CSM by this example
   They are defined in Example_Flash2806x_CsmKeys.asm
--------------------------------------------------------------------------*/
extern Uint16 PRG_key0;        //   CSM Key values
extern Uint16 PRG_key1;
extern Uint16 PRG_key2;
extern Uint16 PRG_key3;
extern Uint16 PRG_key4;
extern Uint16 PRG_key5;
extern Uint16 PRG_key6;
extern Uint16 PRG_key7;

//---------------------------------------------------------------------------
// Common CPU Definitions used by this example:
//

#define	 EALLOW	asm(" EALLOW")
#define	 EDIS	asm(" EDIS")
#define  DINT   asm(" setc INTM")

Uint16 Example_CsmUnlock()
{
    volatile Uint16 temp;

    // Load the key registers with the current password
    // These are defined in Example_Flash2806x_CsmKeys.asm

    EALLOW;
    CsmRegs.KEY0 = PRG_key0;
    CsmRegs.KEY1 = PRG_key1;
    CsmRegs.KEY2 = PRG_key2;
    CsmRegs.KEY3 = PRG_key3;
    CsmRegs.KEY4 = PRG_key4;
    CsmRegs.KEY5 = PRG_key5;
    CsmRegs.KEY6 = PRG_key6;
    CsmRegs.KEY7 = PRG_key7;
    EDIS;

    // Perform a dummy read of the password locations
    // if they match the key values, the CSM will unlock

    temp = CsmPwl.PSWD0;
    temp = CsmPwl.PSWD1;
    temp = CsmPwl.PSWD2;
    temp = CsmPwl.PSWD3;
    temp = CsmPwl.PSWD4;
    temp = CsmPwl.PSWD5;
    temp = CsmPwl.PSWD6;
    temp = CsmPwl.PSWD7;

    // If the CSM unlocked, return succes, otherwise return
    // failure.
    if ( (CsmRegs.CSMSCR.all & 0x0001) == 0) return STATUS_SUCCESS;
    else return STATUS_FAIL_CSM_LOCKED;

}

static int verify_checksum(Uint16 *start_addr)
{
	Uint16 i;
	Uint16 checksum = 0;
	for (i = 0; i < (WORDS_IN_FLASH_BUFFER-1); i++) {
		checksum += start_addr[i];
	}
	if (checksum == start_addr[i]) return 1;
	return 0;
}

static void make_checksum(Uint16 *start_addr)
{
	Uint16 i;
	Uint16 checksum = 0;
	for (i = 0; i < (WORDS_IN_FLASH_BUFFER-1); i++) {
		checksum += start_addr[i];
	}
	start_addr[i] = checksum;
}


int erase_our_flash()
{
	Uint16  Status;
	Uint16  VersionHex;     // Version of the API in decimal encoded hex
	EALLOW;
	Flash_CPUScaleFactor = SCALE_FACTOR;
	EDIS;

	VersionHex = Flash_APIVersionHex();
	if(VersionHex != 0x0100)
	{
	    // Unexpected API version
	    // Make a decision based on this info.
	    asm("    ESTOP0");
	}

	Example_CsmUnlock();
	/* only need to erase B, everything else will be erased again later. */
	Status = Flash_Erase(SECTORB, &FlashStatus);
	Status = Flash_Erase(SECTORG, &FlashStatus);
	if (Status != STATUS_SUCCESS) {
		return -1;
	}
	return 1;
}

int write_flash(void)
{
	Uint16  i;
	Uint16  Status;
	Uint16  *Flash_ptr;     // Pointer to a location in flash
	Uint32  Length;         // Number of 16-bit values to be programmed
	Uint16  VersionHex;     // Version of the API in decimal encoded hex

	EALLOW;
	Flash_CPUScaleFactor = SCALE_FACTOR;
	EDIS;

	VersionHex = Flash_APIVersionHex();
	if(VersionHex != 0x0100)
	{
	    // Unexpected API version
	    // Make a decision based on this info.
	    asm("    ESTOP0");
	}

	Example_CsmUnlock();

	Status = Flash_Erase(SECTORH, &FlashStatus);
	if (Status != STATUS_SUCCESS) {
		return -1;
	}
    for(i=0;i<WORDS_IN_FLASH_BUFFER;i++)
    {
        Buffer[i] = ((Uint16 *)&flash_params)[i];
    }
    make_checksum(Buffer);
    Flash_ptr = (Uint16 *)START_ADDR;
    Length = WORDS_IN_FLASH_BUFFER*sizeof(Buffer[0]);
    Status = Flash_Program(Flash_ptr,Buffer,Length,&FlashStatus);
    if(Status != STATUS_SUCCESS)
    {
    	return -2;
    }
    return 1;
}

int init_flash(void)
{
	if (sizeof(flash_params) >= (sizeof(Buffer[0])*(WORDS_IN_FLASH_BUFFER-1))) {
		// this is an error that needs to be resolved at compile time
		while (1);
	}

	if (verify_checksum((Uint16 *)START_ADDR)) {
		// copy to ram
		memcpy(&flash_params,(Uint16 *)START_ADDR,sizeof(flash_params));
		return 1;
	}

	return -1;
}
