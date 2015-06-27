
#include "flash/flash.h"
#include "Flash2806x_API_Library.h"
#include "F2806x_SysCtrl.h"

typedef union {
	Uint32 uint32_val;
	float float_val;
} IntOrFloat;

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

	// Flash verification will fail if the *size* of the struct has changed
	// If the *size* has stayed the same but the order has changed, incorrect params could be loaded from flash
	if (verify_checksum((Uint16 *)START_ADDR)) {
		// Copy the parameters in flash into a shadow struct first
		struct flash_param_struct_0001 flash_params_shadow;
		memcpy(&flash_params_shadow, (Uint16 *)START_ADDR, sizeof(flash_params_shadow));

		// Run a parameter migration if the struct id has changed
		// This relies on the first 32 bit float of the flash param struct *always* being the id
		if(flash_params_shadow.flash_struct_id != flash_params.flash_struct_id) {

			// Declare all older param structs outside of the switch case
			struct flash_param_struct_0000 flash_params_old = {0};

			// Handle flash param migrations *from* the id stored in flash *to* this version of the compiled firmware
			switch(flash_params_shadow.flash_struct_id) {

				// Last seen in 047d3dfda2072b3d7c8d4143330aabe7e7c72bb0
				case 0x0000:
					// Load the struct from flash into the old struct layout
					memcpy(&flash_params_old, (Uint16 *)START_ADDR, sizeof(flash_params_old));

					// The following parameters were stored as uint32_t in the version 0x0000 struct
					IntOrFloat float_converter;
					float_converter.uint32_val = flash_params_old.ser_num_1;
					flash_params.ser_num_1 = float_converter.float_val;

					float_converter.uint32_val = flash_params_old.ser_num_2;
					flash_params.ser_num_2 = float_converter.float_val;

					float_converter.uint32_val = flash_params_old.ser_num_3;
					flash_params.ser_num_3 = float_converter.float_val;

					float_converter.uint32_val = flash_params_old.assy_time;
					flash_params.assy_time = float_converter.float_val;

					// Copy floats
					flash_params.broadcast_msgs = flash_params_old.broadcast_msgs;
					flash_params.k_rate = flash_params_old.k_rate;

					// Copy arrays
					memcpy(flash_params.commutation_slope, flash_params_old.AxisCalibrationSlopes, sizeof(flash_params_old.AxisCalibrationSlopes) * AXIS_CNT);
					memcpy(flash_params.commutation_icept, flash_params_old.AxisCalibrationIntercepts, sizeof(flash_params_old.AxisCalibrationIntercepts) * AXIS_CNT);

					memcpy(flash_params.torque_pid_kp, flash_params_old.torque_pid_kp, sizeof(flash_params_old.torque_pid_kp) * AXIS_CNT);
					memcpy(flash_params.torque_pid_ki, flash_params_old.torque_pid_ki, sizeof(flash_params_old.torque_pid_ki) * AXIS_CNT);
					memcpy(flash_params.torque_pid_kd, flash_params_old.torque_pid_kd, sizeof(flash_params_old.torque_pid_kd) * AXIS_CNT);

					memcpy(flash_params.rate_pid_p, flash_params_old.rate_pid_p, sizeof(flash_params_old.rate_pid_p) * AXIS_CNT);
					memcpy(flash_params.rate_pid_i, flash_params_old.rate_pid_i, sizeof(flash_params_old.rate_pid_i) * AXIS_CNT);
					memcpy(flash_params.rate_pid_d, flash_params_old.rate_pid_d, sizeof(flash_params_old.rate_pid_d) * AXIS_CNT);
					memcpy(flash_params.rate_pid_windup, flash_params_old.rate_pid_windup, sizeof(flash_params_old.rate_pid_windup) * AXIS_CNT);

					memcpy(flash_params.offset_joint, flash_params_old.offset_joint, sizeof(flash_params_old.offset_joint) * AXIS_CNT);
					memcpy(flash_params.offset_accelerometer, flash_params_old.offset_accelerometer, sizeof(flash_params_old.offset_accelerometer) * AXIS_CNT);
					memcpy(flash_params.offset_gyro, flash_params_old.offset_gyro, sizeof(flash_params_old.offset_gyro) * AXIS_CNT);

					/* Deleted parameters:
					 * 	board_id
					 * 	other_id
					 * 	assy_date
					 * 	mavlink_baud_rate
					 * 	AxisHomePositions
					 * 	pos_pid_p
					 * 	pos_pid_i
					 * 	pos_pid_d
					 * 	pos_pid_windup
					 * 	balance_axis
					 * 	balance_step_duration
					 */
					break;

				// Migrations are one only forwards compatible. When loading an older firmware it's
				//	not possible to determine the unknown newer struct layout, so instead of
				//	potentially harming hardware, the software erase the incompatible firmware and
				//	reset into the bootloader.
				// This could be handled better, but downgrades should be rare to non-existent.
				default:
					erase_our_flash();

					// reset
					extern void WDogEnable(void);
					WDogEnable();

					EALLOW;
					// Cause a device reset by writing incorrect values into WDCHK
					SysCtrlRegs.WDCR = 0x0010;
					EDIS;

					// This should never be reached.
					for(;;) {};
			}

			// Save the new param struct to flash
			write_flash();
		}

		// Copy parameters from flash to ram
		memcpy(&flash_params, (Uint16 *)START_ADDR, sizeof(flash_params));
		return 1;
	}

	return -1;
}
