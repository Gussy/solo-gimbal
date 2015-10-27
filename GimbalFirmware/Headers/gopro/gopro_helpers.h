#ifndef _GOPRO_HELPERS_H
#define _GOPRO_HELPERS_H

#include "f2806x_int8.h"
#include <inttypes.h>
#include <stdbool.h>
#include <time.h>

#include "gopro_interface.h"

bool gp_send_mav_can_response(const gp_transaction_t *t);

time_t gp_time_from_mav(const gp_can_mav_set_req_t* request);
void gp_time_to_mav(gp_can_mav_get_rsp_t *rsp, const struct tm *ti);

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
