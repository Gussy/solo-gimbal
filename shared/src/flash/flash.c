#include "memory_map.h"
#include "flash/flash.h"
#include "flash/flash_helpers.h"
#include "flash/flash_migrations.h"
#include "parameters/flash_params.h"
#include "Flash2806x_API_Library.h"
#include "F2806x_SysCtrl.h"
#include "F2806x_Device.h"

static FLASH_ST FlashStatus;

// Buffer used for calculating flash checksums
#define  WORDS_IN_FLASH_BUFFER 0x100
static Uint16  Buffer[WORDS_IN_FLASH_BUFFER];

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

	flash_csm_unlock();
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

    flash_csm_unlock();
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

	flash_csm_unlock();

	Status = Flash_Erase(SECTORH, &FlashStatus);
	if (Status != STATUS_SUCCESS) {
		return -1;
	}
    for(i=0;i<WORDS_IN_FLASH_BUFFER;i++)
    {
        Buffer[i] = ((Uint16 *)&flash_params)[i];
    }
    make_checksum(Buffer);
    Flash_ptr = (Uint16 *)PARAMS_START;
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
	if (verify_checksum((Uint16 *)PARAMS_START)) {
		// Get the current flash_struct_id from flash
	    Uint16 current_flash_param_rev = 0xFFFF;
		memcpy(&current_flash_param_rev, (Uint16 *)PARAMS_START, sizeof(Uint16));

		// Run a parameter migration if the struct id has changed
		// This relies on the first 16 bit int of the flash param struct *always* being the id
		if(current_flash_param_rev != flash_params.flash_struct_id) {
		    // Run the flash migration operation
		    flash_migration_run(current_flash_param_rev);

			// Save the new param struct to flash
			write_flash();
		}

		// Copy parameters from flash to ram
		memcpy(&flash_params, (Uint16 *)PARAMS_START, sizeof(flash_params));
		return 1;
	}

	return -1;
}

Uint16 compute_flash_params_checksum(void)
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
