#include "memory_map.h"
#include "flash/flash.h"
#include "flash/flash_helpers.h"
#include "flash/flash_migrations.h"
#include "parameters/flash_params.h"
#include "parameters/kvstore.h"
#include "Flash2806x_API_Library.h"
#include "F2806x_SysCtrl.h"
#include "F2806x_Device.h"

static FLASH_ST FlashStatus;

// Buffer used for calculating flash checksums
#define  WORDS_IN_FLASH_BUFFER 0x100
//static Uint16  Buffer[WORDS_IN_FLASH_BUFFER];

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

int write_flash(void)
{
	//TODO: remove usage of this function from codebase, once journaling is complete
    kvstore_save();
    return 1;
}

int init_flash(void)
{
    kvstore_header_t kvstore_header = {0};

	// Initialise the kvstore
	kvstore_init();

	/* Load the header of the kvstore to determine if a valid kvstore has been saved */
	kvstore_get_header(&kvstore_header);

	if(kvstore_header.magic == 0xFFFF) { // Ignore an unprogrammed kvstore (0xFFFF)
	    return 1;
	} else if(kvstore_header.magic == KVSTORE_HEADER_MAGIC) {
        // Load the kvstore if it already exists
        kvstore_load();
        return 1;
    } else {
	    // Flash verification will fail only if the flash has become corrupt, in which case no migrations can take place
        if (verify_checksum((Uint16 *)PARAMS_START)) {
            // Run the flash migration operation. 'kvstore_header.magic' is the same as the old flash param struct version
            flash_migration_run(kvstore_header.magic);

            // Save the kvstore
            kvstore_save();

            return 1;
        }
	}

	return -1;
}
