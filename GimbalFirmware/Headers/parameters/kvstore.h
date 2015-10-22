#ifndef HEADERS_PARAMETERS_KVSTORE_H_
#define HEADERS_PARAMETERS_KVSTORE_H_

#include "f2806x_int8.h"

#define KVSTORE_HEADER_MAGIC 0xA5A5

typedef enum {
    FLASH_PARAM_STRUCT_ID = 0,
    FLASH_PARAM_ASSY_TIME = 1,
    FLASH_PARAM_SER_NUM_1 = 2,
    FLASH_PARAM_SER_NUM_2 = 3,
    FLASH_PARAM_SER_NUM_3 = 4,
    FLASH_PARAM_K_RATE = 5,
    FLASH_PARAM_COMMUTATION_SLOPE_AZ = 6,
    FLASH_PARAM_COMMUTATION_SLOPE_ROLL = 7,
    FLASH_PARAM_COMMUTATION_SLOPE_EL = 8,
    FLASH_PARAM_COMMUTATION_ICEPT_AZ = 9,
    FLASH_PARAM_COMMUTATION_ICEPT_ROLL = 10,
    FLASH_PARAM_COMMUTATION_ICEPT_EL = 11,
    FLASH_PARAM_TORQUE_PID_KP = 12,
    FLASH_PARAM_TORQUE_PID_KI = 13,
    FLASH_PARAM_TORQUE_PID_KR = 14,
    FLASH_PARAM_RATE_PID_P_AZ = 15,
    FLASH_PARAM_RATE_PID_P_ROLL = 16,
    FLASH_PARAM_RATE_PID_P_EL = 17,
    FLASH_PARAM_RATE_PID_I_AZ = 18,
    FLASH_PARAM_RATE_PID_I_ROLL = 19,
    FLASH_PARAM_RATE_PID_I_EL = 20,
    FLASH_PARAM_RATE_PID_D_AZ = 21,
    FLASH_PARAM_RATE_PID_D_ROLL = 22,
    FLASH_PARAM_RATE_PID_D_EL = 23,
    FLASH_PARAM_RATE_PID_D_ALPHA_AZ = 24,
    FLASH_PARAM_RATE_PID_D_ALPHA_ROLL = 25,
    FLASH_PARAM_RATE_PID_D_ALPHA_EL = 26,
    FLASH_PARAM_OFFSET_JOINT_X = 27,
    FLASH_PARAM_OFFSET_JOINT_Y = 28,
    FLASH_PARAM_OFFSET_JOINT_Z = 29,
    FLASH_PARAM_OFFSET_GYRO_X = 30,
    FLASH_PARAM_OFFSET_GYRO_Y = 31,
    FLASH_PARAM_OFFSET_GYRO_Z = 32,
    FLASH_PARAM_OFFSET_ACCELEROMETER_X = 33,
    FLASH_PARAM_OFFSET_ACCELEROMETER_Y = 34,
    FLASH_PARAM_OFFSET_ACCELEROMETER_Z = 35,
    FLASH_PARAM_GAIN_ACCELEROMETER_X = 36,
    FLASH_PARAM_GAIN_ACCELEROMETER_Y = 37,
    FLASH_PARAM_GAIN_ACCELEROMETER_Z = 38,
    FLASH_PARAM_ALIGNMENT_ACCELEROMETER_X = 39,
    FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Y = 40,
    FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Z = 41,
    FLASH_PARAM_GOPRO_CHARGING_ENABLED = 42,
    FLASH_PARAM_USE_CUSTOM_GAINS = 43,
    FLASH_PARAM_GOPRO_CONTROL = 44,
    FLASH_PARAM_KEY_COUNT
} flash_param_keys_t;

typedef union {
    float as_float;
    uint16_t as_words[2];
    uint32_t as_dword;
    uint8_t as_bytes[4];
} kv_value_t;

typedef struct {
    flash_param_keys_t key;
    kv_value_t value;
} keyvalue_t;

typedef union {
    uint16_t magic;
    uint16_t unused[2];
} kvstore_header_t;

void kvstore_init(void);
void kvstore_load(void);
int16_t kvstore_save(void);
int16_t kvstore_reset(void);

void kvstore_get_header(kvstore_header_t* header);

kv_value_t kvstore_get_value(const flash_param_keys_t key);
float kvstore_get_float(const flash_param_keys_t key);
uint16_t kvstore_get_uint16(const flash_param_keys_t key);

void kvstore_put_value(const flash_param_keys_t key, const kv_value_t value);
void kvstore_put_float(const flash_param_keys_t key, const float value);
void kvstore_put_uint16(const flash_param_keys_t key, const uint16_t value);

#endif /* HEADERS_PARAMETERS_KVSTORE_H_ */
