#include "PM_Sensorless.h"

#include "memory_map.h"
#include "flash/flash.h"
#include "flash/flash_helpers.h"
#include "parameters/kvstore.h"
#include "parameters/flash_params.h"

static FLASH_ST flash_status;

#define KVSTORE_HEADER_WORDS sizeof(kvstore_header_t)
#define KVSTORE_KV_WORDS sizeof(keyvalue_t)

// Buffer used for calculating flash checksums
#define WORDS_IN_FLASH_BUFFER (KVSTORE_HEADER_WORDS + (FLASH_PARAM_KEY_COUNT * 3))
static uint16_t flash_buffer[WORDS_IN_FLASH_BUFFER];

static kv_value_t kvstore[FLASH_PARAM_KEY_COUNT];

static void kvstore_set_defaults(void);

void kvstore_init(void)
{
    uint16_t key;
    for(key = 0; key < FLASH_PARAM_KEY_COUNT; key++) {
        kvstore[key].as_dword = 0;
    }

    // Set the default values
    kvstore_set_defaults();
}

void kvstore_load(void)
{
    uint16_t i, key;

    // Load the kvstore from flash into the flat uint16_t buffer
    memset(&flash_buffer, 0, sizeof(flash_buffer));
    memcpy(&flash_buffer, (Uint16 *)PARAMS_START, sizeof(flash_buffer));

    // Extract keys and values from the flat uint16_t buffer
    for(i = 0; i < FLASH_PARAM_KEY_COUNT; i++) {
        key = flash_buffer[KVSTORE_HEADER_WORDS + (i * KVSTORE_KV_WORDS) + 0];
        kvstore[key].as_words[0] = flash_buffer[KVSTORE_HEADER_WORDS + (i * KVSTORE_KV_WORDS) + 1];
        kvstore[key].as_words[1] = flash_buffer[KVSTORE_HEADER_WORDS + (i * KVSTORE_KV_WORDS) + 2];
    }
}

int16_t kvstore_save(void)
{
    uint16_t key, result, version_hex;
    uint16_t *flash_ptr;
    kvstore_header_t kvstore_header = {0};

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

    // Copy the kvstore header into the buffer
    kvstore_header.magic = KVSTORE_HEADER_MAGIC;
    memcpy(&flash_buffer, &kvstore_header, sizeof(kvstore_header));

    // Copy keys and values into a flat uint16_t buffer
    for(key = 0; key < FLASH_PARAM_KEY_COUNT; key++) {
        flash_buffer[KVSTORE_HEADER_WORDS + (key * KVSTORE_KV_WORDS) + 0] = key;
        flash_buffer[KVSTORE_HEADER_WORDS + (key * KVSTORE_KV_WORDS) + 1] = kvstore[key].as_words[0];
        flash_buffer[KVSTORE_HEADER_WORDS + (key * KVSTORE_KV_WORDS) + 2] = kvstore[key].as_words[1];
    }

    // Write the buffer into flash
    flash_ptr = (uint16_t *)PARAMS_START;
    result = Flash_Program(flash_ptr, flash_buffer, WORDS_IN_FLASH_BUFFER*sizeof(flash_buffer[0]), &flash_status);
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

void kvstore_get_header(kvstore_header_t* header)
{
    memcpy(header, (Uint16 *)PARAMS_START, sizeof(kvstore_header_t));
}

kvstore_state_t kvstore_state(void)
{
    kvstore_header_t kvstore_header = {0};
    kvstore_get_header(&kvstore_header);

    switch(kvstore_header.magic) {
        case 0xFFFF:
            return KVSTORE_EMPTY;

        case KVSTORE_HEADER_MAGIC:
            return KVSTORE_EXISTS;

        default:
            return KVSTORE_NOT_MIGRATED;

    }
}

kv_value_t kvstore_get_value(const flash_param_keys_t key)
{
    kv_value_t result = {0};

    if(key < FLASH_PARAM_KEY_COUNT) {
        result = kvstore[key];
    }

    return result;
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

void kvstore_put_value(const flash_param_keys_t key, const kv_value_t value)
{
    if(key < FLASH_PARAM_KEY_COUNT) {
        kvstore[key] = value;
    }
}

void kvstore_put_float(const flash_param_keys_t key, const float value)
{
    if(key < FLASH_PARAM_KEY_COUNT) {
        kvstore[key].as_float = value;
    }
}

void kvstore_put_uint16(const flash_param_keys_t key, const uint16_t value)
{
    if(key < FLASH_PARAM_KEY_COUNT) {
        kvstore[key].as_words[0] = value;
    }
}
