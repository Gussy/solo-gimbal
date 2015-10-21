#include "PM_Sensorless.h"

#include "parameters/kvstore.h"
#include "parameters/flash_params.h"

static kv_value_t kvstore[FLASH_PARAM_KEY_COUNT];

void kvstore_init(void)
{
    uint16_t i;
    for(i = 0; i < FLASH_PARAM_KEY_COUNT; i++) {
        memset(&kvstore[i].as_bytes, 0, sizeof(kvstore[i].as_bytes));
    }

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
