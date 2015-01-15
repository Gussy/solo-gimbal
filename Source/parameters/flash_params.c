/*
 * flash_params.c
 *
 *  Created on: Jan 5, 2015
 *      Author: ksmith
 */


/*---- Flash API include file -------------------------------------------------*/
//#define FLASH_F28062 1

#include "Flash2806x_API_Library.h"

#include "parameters/flash_params.h"

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

#define FLASH_END_ADDR    0x3F7FFF

extern SECTOR Sector[8];

#if (FLASH_F28069 || FLASH_F28068 || FLASH_F28067 || FLASH_F28066)
#define FLASH_START_ADDR  0x3D8000
SECTOR Sector[8]= {
         (Uint16 *) 0x3D8000,(Uint16 *) 0x3DBFFF,
         (Uint16 *) 0x3DC000,(Uint16 *) 0x3DFFFF,
         (Uint16 *) 0x3E0000,(Uint16 *) 0x3E3FFF,
         (Uint16 *) 0x3E4000,(Uint16 *) 0x3E7FFF,
         (Uint16 *) 0x3E8000,(Uint16 *) 0x3EBFFF,
         (Uint16 *) 0x3EC000,(Uint16 *) 0x3EFFFF,
         (Uint16 *) 0x3F0000,(Uint16 *) 0x3F3FFF,
         (Uint16 *) 0x3F4000,(Uint16 *) 0x3F7FFF,
};

#elif (FLASH_F28065 || FLASH_F28064 || FLASH_F28063 || FLASH_F28062)
#define FLASH_START_ADDR  0x3E8000
SECTOR Sector[8]= {
         (Uint16 *) 0x3E8000,(Uint16 *) 0x3E9FFF,
         (Uint16 *) 0x3EA000,(Uint16 *) 0x3EBFFF,
         (Uint16 *) 0x3EC000,(Uint16 *) 0x3EDFFF,
         (Uint16 *) 0x3EE000,(Uint16 *) 0x3EFFFF,
         (Uint16 *) 0x3F0000,(Uint16 *) 0x3F1FFF,
         (Uint16 *) 0x3F2000,(Uint16 *) 0x3F3FFF,
         (Uint16 *) 0x3F4000,(Uint16 *) 0x3F5FFF,
         (Uint16 *) 0x3F6000,(Uint16 *) 0x3F7FFF,
};
#endif


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

// TODO: This is temporary for development purposes until we start loading the calibration parameters in flash
#define PROTOTYPE_HW 2 // 1 is old hardware, prototype Arthur has, 2 is new hardware, prototype Aaron has

#if (PROTOTYPE_HW == 1)
struct flash_param_struct_0000 flash_params =
{
    0x0000,                     // Flash Struct ID
    0,                          // Board ID
    0,                          // Other ID
    0x0000,                     // Software version number TODO: populate this from git version info
    115,                        // Mavlink baud rate
    // Axis calibration slopes
    {
        0.127,      // EL
        0.1267,     // AZ
        0.1274,     // ROLL
    },
    // Axis calibration intercepts
    {
        0.3801,     // EL
        0.4128,     // AZ
        0.43,       // ROLL
    },
    // Axis home positions
    {
        5135,   // EL
        4696,   // AZ
        4319,   // ROLL
    },
    // Rate PID P gains
    {
        2.5,    // EL
        2.0,    // AZ
        5.0     // ROLL
    },
    // Rate PID I gains
    {
        0.25,   // EL
        0.5,    // AZ
        0.5     // ROLL
    },
    // Rate PID D gains
    {
        0.0,    // EL
        1.0,    // AZ
        0.0     // ROLL
    },
    // Rate PID windup limits
    {
        32768.0,// EL
        32768.0,// AZ
        32768.0 // ROLL
    },
    // Position PID P gains
    {
        1.0, // EL
        1.0, // AZ
        1.0  // ROLL
    },
    // Position PID I gains
    {
        0.0, // EL
        0.0, // AZ
        0.0  // ROLL
    },
    // Position PID D gains
    {
        0.0, // EL
        0.0, // AZ
        0.0  // ROLL
    },
    // Position PID windup limits
    {
        2000.0, // EL
        2000.0, // AZ
        2000.0  // ROLL
    },
    // Gyro offsets
    {
        0.0, // EL
        0.0, // AZ
        0.0  // ROLL
    },
    // Torque Loop PID Kp
    {
        1.25,   // EL
        0.8,    // AZ
        0.8     // ROLL
    },
    // Torque Loop PID Ki
    {
        0.75,   // EL
        0.75,   // AZ
        0.75    // ROLL
    },
    // Torque Loop PID Kd
    {
        1.0,    // EL
        0.0,    // AZ
        0.0     // ROLL
    }
};
#elif (PROTOTYPE_HW == 2)
struct flash_param_struct_0000 flash_params =
{
    0x0000,                     // Flash Struct ID
    0,                          // Board ID
    0,                          // Other ID
    0x0000,                     // Software version number TODO: populate this from git version info
    115,                        // Mavlink baud rate
    // Axis calibration slopes
    {
        0.126,        // EL
        0.1247,       // AZ
        0.1245        // ROLL
    },
    // Axis calibration intercepts
    {
        0.4536,      // EL
        0.3718,      // AZ
        0.4079       // ROLL
    },
    // Axis home positions
    {
        5120,      // EL
        4898,      // AZ
        4944       // ROLL
    },
    // Rate PID P gains
    {
        2.5,    // EL
        2.0,    // AZ
        5.0     // ROLL
    },
    // Rate PID I gains
    {
        0.25,   // EL
        0.5,    // AZ
        0.5     // ROLL
    },
    // Rate PID D gains
    {
        0.0,    // EL
        1.0,    // AZ
        0.0     // ROLL
    },
    // Rate PID windup limits
    {
        32768.0,// EL
        32768.0,// AZ
        32768.0 // ROLL
    },
    // Position PID P gains
	{
		1.0, // EL
		1.0, // AZ
		1.0  // ROLL
	},
	// Position PID I gains
	{
		0.0, // EL
		0.0, // AZ
		0.0  // ROLL
	},
	// Position PID D gains
	{
		0.0, // EL
		0.0, // AZ
		0.0  // ROLL
	},
	// Position PID windup limits
	{
		2000.0, // EL
		2000.0, // AZ
		2000.0  // ROLL
	},
	// Gyro offsets
	{
		0.0, // EL
		0.0, // AZ
		0.0  // ROLL
	},
    // Torque Loop PID Kp
    {
        0.8,   // EL
        0.8,    // AZ
        0.8     // ROLL
    },
    // Torque Loop PID Ki
    {
        0.75,   // EL
        0.75,   // AZ
        0.75    // ROLL
    },
    // Torque Loop PID Kd
    {
        0.0,    // EL
        0.0,    // AZ
        0.0     // ROLL
    }
};
#endif


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


int init_flash(void)
{
	if (sizeof(flash_params) >= (sizeof(Buffer[0])*(WORDS_IN_FLASH_BUFFER-1))) {
		// this is an error that needs to be resolved at compile time
		while (1);
	}
#define START_ADDR 0x3E8000
	if (verify_checksum((Uint16 *)START_ADDR)) {
		// copy to ram
		memcpy(&flash_params,(Uint16 *)START_ADDR,sizeof(flash_params));
		return 1;
	}
	write_flash();

	return -1;
}

Uint16 PRG_key0 = 0xFFFF;
Uint16 PRG_key1 = 0xFFFF;
Uint16 PRG_key2 = 0xFFFF;
Uint16 PRG_key3 = 0xFFFF;
Uint16 PRG_key4 = 0xFFFF;
Uint16 PRG_key5 = 0xFFFF;
Uint16 PRG_key6 = 0xFFFF;
Uint16 PRG_key7 = 0xFFFF;

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


int write_flash(void)
{
	Uint16  i;
	Uint16  Status;
	Uint16  *Flash_ptr;     // Pointer to a location in flash
	Uint32  Length;         // Number of 16-bit values to be programmed
	Uint16  VersionHex;     // Version of the API in decimal encoded hex

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

/*------------------------------------------------------------------
  Callback function - must be executed from outside flash/OTP
-----------------------------------------------------------------*/
#pragma CODE_SECTION(MyCallbackFunction,"ramfuncs");
void MyCallbackFunction(void)
{
    // Toggle pin, service external watchdog etc
    MyCallbackCounter++;
    asm("    NOP");
}
