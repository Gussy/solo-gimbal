#ifndef _GOPRO_HELPERS_H
#define _GOPRO_HELPERS_H

#include "f2806x_int8.h"
#include <inttypes.h>
#include <stdbool.h>

bool gp_send_get_response(uint8_t cmd_id, uint8_t value);
bool gp_send_set_response(uint8_t cmd_id, uint8_t result);

// get and set state of bp_detect pin
void gp_set_bp_detect_asserted_out(bool assert);
bool gp_get_bp_detect_asserted_out(void);

// get and set state of intr pin
void gp_set_intr_asserted_out(bool assert);
bool gp_get_intr_asserted_out(void);

// get and set state of pwron pin
void gp_set_pwron_asserted_out(bool assert);
bool gp_get_pwron_asserted_out(void);

// get and set state of charging pin
void gp_set_charging_asserted_out(bool assert);
bool gp_get_charging_asserted_out(void);

// get state of von pin
bool gp_get_von_asserted_in(void);

#endif // _GOPRO_HELPERS_H
