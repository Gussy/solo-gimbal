#include "PM_Sensorless.h"
#include "flash/flash.h"
#include "hardware/watchdog.h"
#include "can/cand.h"
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

int erase_param_flash()
{
    Uint16  Status;
    Uint16  VersionHex;     // Version of the API in decimal encoded hex
    EALLOW;
    Flash_CPUScaleFactor = SCALE_FACTOR;
    EDIS;

    VersionHex = Flash_APIVersionHex();
    if(VersionHex != 0x0100) {
        // Unexpected API version
        // Make a decision based on this info.
        asm("    ESTOP0");
    }

    Example_CsmUnlock();
    Status = Flash_Erase(SECTORH, &FlashStatus);
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

	// Flash verification will fail only if the flash has become corrupt, in which case no migrations can take place
	if (verify_checksum((Uint16 *)START_ADDR)) {
		// Copy the parameters in flash into a shadow struct first
		struct CURRENT_FLASH_PARAM_STRUCT flash_params_shadow;
		memcpy(&flash_params_shadow, (Uint16 *)START_ADDR, sizeof(flash_params_shadow));

		// Run a parameter migration if the struct id has changed
		// This relies on the first 16 bit int of the flash param struct *always* being the id
		if(flash_params_shadow.flash_struct_id != flash_params.flash_struct_id) {

			// Declare all older param structs outside of the switch case
			struct flash_param_struct_0000 flash_params_0000 = {0};
			struct flash_param_struct_0001 flash_params_0001 = {0};
			struct flash_param_struct_0002 flash_params_0002 = {0};
			struct flash_param_struct_0003 flash_params_0003 = {0};

			// Handle flash param migrations *from* the id stored in flash *to* this version of the compiled firmware
			switch(flash_params_shadow.flash_struct_id) {
				case 0x0003:
					// Load the struct from flash into the old struct layout
					memcpy(&flash_params_0003, (Uint16 *)START_ADDR, sizeof(flash_params_0003));

					// Copy floats
					flash_params.ser_num_1 = flash_params_0003.ser_num_1;
					flash_params.ser_num_2 = flash_params_0003.ser_num_2;
					flash_params.ser_num_3 = flash_params_0003.ser_num_3;
					flash_params.assy_time = flash_params_0003.assy_time;
					flash_params.k_rate = flash_params_0003.k_rate;
					flash_params.gopro_charging_enabled = flash_params_0003.gopro_charging_enabled;

					// Copy arrays
					memcpy(flash_params.commutation_slope, flash_params_0003.commutation_slope, sizeof(flash_params_0003.commutation_slope) * AXIS_CNT);
					memcpy(flash_params.commutation_icept, flash_params_0003.commutation_icept, sizeof(flash_params_0003.commutation_icept) * AXIS_CNT);

					memcpy(flash_params.torque_pid_kp, flash_params_0003.torque_pid_kp, sizeof(flash_params_0003.torque_pid_kp) * AXIS_CNT);
					memcpy(flash_params.torque_pid_ki, flash_params_0003.torque_pid_ki, sizeof(flash_params_0003.torque_pid_ki) * AXIS_CNT);
					memcpy(flash_params.torque_pid_kd, flash_params_0003.torque_pid_kd, sizeof(flash_params_0003.torque_pid_kd) * AXIS_CNT);

					memcpy(flash_params.rate_pid_p, flash_params_0003.rate_pid_p, sizeof(flash_params_0003.rate_pid_p) * AXIS_CNT);
					memcpy(flash_params.rate_pid_i, flash_params_0003.rate_pid_i, sizeof(flash_params_0003.rate_pid_i) * AXIS_CNT);
					memcpy(flash_params.rate_pid_d, flash_params_0003.rate_pid_d, sizeof(flash_params_0003.rate_pid_d) * AXIS_CNT);
					memcpy(flash_params.rate_pid_windup, flash_params_0003.rate_pid_windup, sizeof(flash_params_0003.rate_pid_windup) * AXIS_CNT);

					memcpy(flash_params.offset_joint, flash_params_0003.offset_joint, sizeof(flash_params_0003.offset_joint) * AXIS_CNT);
					memcpy(flash_params.offset_accelerometer, flash_params_0003.offset_accelerometer, sizeof(flash_params_0003.offset_accelerometer) * AXIS_CNT);
					memcpy(flash_params.offset_gyro, flash_params_0003.offset_gyro, sizeof(flash_params_0003.offset_gyro) * AXIS_CNT);

					memcpy(flash_params.gain_accelerometer, flash_params_0003.gain_accelerometer, sizeof(flash_params_0003.gain_accelerometer) * AXIS_CNT);
					memcpy(flash_params.offset_accelerometer, flash_params_0003.offset_accelerometer, sizeof(flash_params_0003.gain_accelerometer) * AXIS_CNT);

					/* Removed parameters:
					 * 	broadcast_msgs (was made volatile)
					 */
					break;

				case 0x0002:
					// Load the struct from flash into the old struct layout
					memcpy(&flash_params_0002, (Uint16 *)START_ADDR, sizeof(flash_params_0002));

					// Copy floats
					flash_params.ser_num_1 = flash_params_0002.ser_num_1;
					flash_params.ser_num_2 = flash_params_0002.ser_num_2;
					flash_params.ser_num_3 = flash_params_0002.ser_num_3;
					flash_params.assy_time = flash_params_0002.assy_time;
					flash_params.k_rate = flash_params_0002.k_rate;

					// Copy arrays
					memcpy(flash_params.commutation_slope, flash_params_0002.commutation_slope, sizeof(flash_params_0002.commutation_slope) * AXIS_CNT);
					memcpy(flash_params.commutation_icept, flash_params_0002.commutation_icept, sizeof(flash_params_0002.commutation_icept) * AXIS_CNT);

					memcpy(flash_params.torque_pid_kp, flash_params_0002.torque_pid_kp, sizeof(flash_params_0002.torque_pid_kp) * AXIS_CNT);
					memcpy(flash_params.torque_pid_ki, flash_params_0002.torque_pid_ki, sizeof(flash_params_0002.torque_pid_ki) * AXIS_CNT);
					memcpy(flash_params.torque_pid_kd, flash_params_0002.torque_pid_kd, sizeof(flash_params_0002.torque_pid_kd) * AXIS_CNT);

					memcpy(flash_params.rate_pid_p, flash_params_0002.rate_pid_p, sizeof(flash_params_0002.rate_pid_p) * AXIS_CNT);
					memcpy(flash_params.rate_pid_i, flash_params_0002.rate_pid_i, sizeof(flash_params_0002.rate_pid_i) * AXIS_CNT);
					memcpy(flash_params.rate_pid_d, flash_params_0002.rate_pid_d, sizeof(flash_params_0002.rate_pid_d) * AXIS_CNT);
					memcpy(flash_params.rate_pid_windup, flash_params_0002.rate_pid_windup, sizeof(flash_params_0002.rate_pid_windup) * AXIS_CNT);

					memcpy(flash_params.offset_joint, flash_params_0002.offset_joint, sizeof(flash_params_0002.offset_joint) * AXIS_CNT);
					memcpy(flash_params.offset_accelerometer, flash_params_0002.offset_accelerometer, sizeof(flash_params_0002.offset_accelerometer) * AXIS_CNT);
					memcpy(flash_params.offset_gyro, flash_params_0002.offset_gyro, sizeof(flash_params_0002.offset_gyro) * AXIS_CNT);

					memcpy(flash_params.gain_accelerometer, flash_params_0002.gain_accelerometer, sizeof(flash_params_0002.gain_accelerometer) * AXIS_CNT);
					memcpy(flash_params.offset_accelerometer, flash_params_0002.offset_accelerometer, sizeof(flash_params_0002.gain_accelerometer) * AXIS_CNT);

					/* Added parameters:
					 *  gopro_charging_enabled
					 */
					break;

				// Last seen in 2e89f5a10768188863f3e456638c7966e72723d3
				case 0x0001:
					// Load the struct from flash into the old struct layout
					memcpy(&flash_params_0001, (Uint16 *)START_ADDR, sizeof(flash_params_0001));

					// Copy floats
					flash_params.ser_num_1 = flash_params_0001.ser_num_1;
					flash_params.ser_num_2 = flash_params_0001.ser_num_2;
					flash_params.ser_num_3 = flash_params_0001.ser_num_3;
					flash_params.assy_time = flash_params_0001.assy_time;
					flash_params.k_rate = flash_params_0001.k_rate;

					// Copy arrays
					memcpy(flash_params.commutation_slope, flash_params_0001.commutation_slope, sizeof(flash_params_0001.commutation_slope) * AXIS_CNT);
					memcpy(flash_params.commutation_icept, flash_params_0001.commutation_icept, sizeof(flash_params_0001.commutation_icept) * AXIS_CNT);

					memcpy(flash_params.torque_pid_kp, flash_params_0001.torque_pid_kp, sizeof(flash_params_0001.torque_pid_kp) * AXIS_CNT);
					memcpy(flash_params.torque_pid_ki, flash_params_0001.torque_pid_ki, sizeof(flash_params_0001.torque_pid_ki) * AXIS_CNT);
					memcpy(flash_params.torque_pid_kd, flash_params_0001.torque_pid_kd, sizeof(flash_params_0001.torque_pid_kd) * AXIS_CNT);

					memcpy(flash_params.rate_pid_p, flash_params_0001.rate_pid_p, sizeof(flash_params_0001.rate_pid_p) * AXIS_CNT);
					memcpy(flash_params.rate_pid_i, flash_params_0001.rate_pid_i, sizeof(flash_params_0001.rate_pid_i) * AXIS_CNT);
					memcpy(flash_params.rate_pid_d, flash_params_0001.rate_pid_d, sizeof(flash_params_0001.rate_pid_d) * AXIS_CNT);
					memcpy(flash_params.rate_pid_windup, flash_params_0001.rate_pid_windup, sizeof(flash_params_0001.rate_pid_windup) * AXIS_CNT);

					memcpy(flash_params.offset_joint, flash_params_0001.offset_joint, sizeof(flash_params_0001.offset_joint) * AXIS_CNT);
					memcpy(flash_params.offset_accelerometer, flash_params_0001.offset_accelerometer, sizeof(flash_params_0001.offset_accelerometer) * AXIS_CNT);
					memcpy(flash_params.offset_gyro, flash_params_0001.offset_gyro, sizeof(flash_params_0001.offset_gyro) * AXIS_CNT);

					/* Added parameters:
					 * 	gain_accelerometer
					 * 	alignment_accelerometer
					 */
					break;

				// Last seen in 047d3dfda2072b3d7c8d4143330aabe7e7c72bb0
				case 0x0000:
					// Load the struct from flash into the old struct layout
					memcpy(&flash_params_0000, (Uint16 *)START_ADDR, sizeof(flash_params_0000));

					// The following parameters were stored as uint32_t in the version 0x0000 struct
					IntOrFloat float_converter;
					float_converter.uint32_val = flash_params_0000.ser_num_1;
					flash_params.ser_num_1 = float_converter.float_val;

					float_converter.uint32_val = flash_params_0000.ser_num_2;
					flash_params.ser_num_2 = float_converter.float_val;

					float_converter.uint32_val = flash_params_0000.ser_num_3;
					flash_params.ser_num_3 = float_converter.float_val;

					float_converter.uint32_val = flash_params_0000.assy_time;
					flash_params.assy_time = float_converter.float_val;

					// Copy floats
					flash_params.k_rate = flash_params_0000.k_rate;

					// Copy arrays
					memcpy(flash_params.commutation_slope, flash_params_0000.AxisCalibrationSlopes, sizeof(flash_params_0000.AxisCalibrationSlopes) * AXIS_CNT);
					memcpy(flash_params.commutation_icept, flash_params_0000.AxisCalibrationIntercepts, sizeof(flash_params_0000.AxisCalibrationIntercepts) * AXIS_CNT);

					memcpy(flash_params.torque_pid_kp, flash_params_0000.torque_pid_kp, sizeof(flash_params_0000.torque_pid_kp) * AXIS_CNT);
					memcpy(flash_params.torque_pid_ki, flash_params_0000.torque_pid_ki, sizeof(flash_params_0000.torque_pid_ki) * AXIS_CNT);
					memcpy(flash_params.torque_pid_kd, flash_params_0000.torque_pid_kd, sizeof(flash_params_0000.torque_pid_kd) * AXIS_CNT);

					memcpy(flash_params.rate_pid_p, flash_params_0000.rate_pid_p, sizeof(flash_params_0000.rate_pid_p) * AXIS_CNT);
					memcpy(flash_params.rate_pid_i, flash_params_0000.rate_pid_i, sizeof(flash_params_0000.rate_pid_i) * AXIS_CNT);
					memcpy(flash_params.rate_pid_d, flash_params_0000.rate_pid_d, sizeof(flash_params_0000.rate_pid_d) * AXIS_CNT);
					memcpy(flash_params.rate_pid_windup, flash_params_0000.rate_pid_windup, sizeof(flash_params_0000.rate_pid_windup) * AXIS_CNT);

					memcpy(flash_params.offset_joint, flash_params_0000.offset_joint, sizeof(flash_params_0000.offset_joint) * AXIS_CNT);
					memcpy(flash_params.offset_accelerometer, flash_params_0000.offset_accelerometer, sizeof(flash_params_0000.offset_accelerometer) * AXIS_CNT);
					memcpy(flash_params.offset_gyro, flash_params_0000.offset_gyro, sizeof(flash_params_0000.offset_gyro) * AXIS_CNT);

					/* Removed parameters:
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
				//	potentially harming hardware, the software will erase the incompatible firmware
				//	and reset into the bootloader.
				// This could be handled better, but downgrades should be rare
				default:
					erase_our_flash();

					// Reset other axes then ourselves
					cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_RESET);
					watchdog_reset();
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

Uint16 compute_flash_params_checksum()
{
	Uint16 checksum = 0;
	Uint16 flash_params_num_words = sizeof(flash_params);
	Uint16* flash_params_ptr = (Uint16*)(&flash_params);
	int i;
	for (i = 0; i < flash_params_num_words; i++) {
		checksum += flash_params_ptr[i];
	}

	return checksum;
}
