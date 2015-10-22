#include "PM_Sensorless.h"

#include "memory_map.h"
#include "flash/flash.h"
#include "flash/flash_helpers.h"
#include "parameters/kvstore.h"
#include "parameters/flash_params.h"

static FLASH_ST flash_status;

// Buffer used for calculating flash checksums
#define WORDS_IN_FLASH_BUFFER (FLASH_PARAM_KEY_COUNT * sizeof(keyvalue_t))
static uint16_t flash_buffer[WORDS_IN_FLASH_BUFFER];

static kv_value_t kvstore[FLASH_PARAM_KEY_COUNT];

static void kvstore_set_defaults(void);

void kvstore_init(void)
{
    uint16_t i;
    for(i = 0; i < FLASH_PARAM_KEY_COUNT; i++) {
        memset(&kvstore[i].as_bytes, 0, sizeof(kvstore[i].as_bytes));
    }

    // Set the default values
    kvstore_set_defaults();
}

void kvstore_load(void)
{
    // TODO: Load kvstore from flash
}

int16_t kvstore_save(void)
{
    uint16_t key;
    uint16_t result;
    uint16_t *flash_ptr;        // Pointer to a location in flash
    uint16_t version_hex;       // Version of the API in decimal encoded hex

    EALLOW;
    Flash_CPUScaleFactor = SCALE_FACTOR;
    EDIS;

    version_hex = Flash_APIVersionHex();
    if(version_hex != 0x0100) {
        // Unexpected API version
        // Make a decision based on this info.
        asm("ESTOP0");
    }

    flash_csm_unlock();

    // Erase existing flash
    result = Flash_Erase(SECTORH, &flash_status);
    if(result != STATUS_SUCCESS) {
        return -1;
    }

    // Copy keys and values into a flat uint16_t buffer
    for(key = 0; key < (WORDS_IN_FLASH_BUFFER / sizeof(keyvalue_t)); key++) {
        flash_buffer[key + 0] = key;
        flash_buffer[key + 1] = kvstore[key].as_words[0];
        flash_buffer[key + 2] = kvstore[key].as_words[1];
    }

    // Write the buffer into flash
    flash_ptr = (Uint16 *)PARAMS_START;
    result = Flash_Program(flash_ptr, flash_buffer, sizeof(flash_buffer), &flash_status);
    if(result != STATUS_SUCCESS) {
        return -2;
    }

    return 1;
}

// Re-initialise the kvstore to all empty values and write to to flash
int16_t kvstore_reset(void)
{
    int16_t result;

    kvstore_init();
    result = kvstore_save();

    return result;
}

static void kvstore_set_defaults(void)
{
    // GoPro charging is enabled by default
    kvstore_put_float(FLASH_PARAM_GOPRO_CHARGING_ENABLED, 1.0f);

    // GoPro control is enabled by default
    kvstore_put_float(FLASH_PARAM_GOPRO_CONTROL, 1.0f);
}

float kvstore_get_float(const flash_param_keys_t key)
{
    if(key < FLASH_PARAM_KEY_COUNT) {
        return kvstore[key].as_float;
    }

    return 0.0f;
}

uint16_t kvstore_get_uint16(const flash_param_keys_t key)
{
    if(key < FLASH_PARAM_KEY_COUNT) {
        return kvstore[key].as_words[0];
    }

    return 0;
}

void kvstore_put_float(const flash_param_keys_t key, const float value)
{
    kvstore[key].as_float = value;
}

void kvstore_put_uint16(const flash_param_keys_t key, const uint16_t value)
{
    kvstore[key].as_words[0] = value;
}
