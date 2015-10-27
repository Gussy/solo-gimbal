#include "gopro/gopro_helpers.h"
#include "can/cand.h"
#include "PeripheralHeaderIncludes.h"


bool gp_send_mav_can_response(const gp_transaction_t *t)
{
    /*
     * Send the mavlink msg back via can to the AZ board,
     * which will then forward it over serial.
     *
     * Relies on the fact that get responses are a superset
     * of set responses, and sends the appropriate number of bytes
     * based on the type of response.
     */

    CAND_Result ret;
    if (t->reqtype == GP_REQUEST_GET) {
        ret = cand_tx_extended_param(CAND_ID_AZ, CAND_PID_GOPRO_GET_RESPONSE, t->response.bytes, sizeof(gp_can_mav_get_rsp_t));
    } else {
        ret = cand_tx_extended_param(CAND_ID_AZ, CAND_PID_GOPRO_SET_RESPONSE, t->response.bytes, sizeof(gp_can_mav_set_rsp_t));
    }

    return (ret == CAND_SUCCESS);
}

// from http://www.catb.org/esr/time-programming
// released to public domain
static time_t timegm(const struct tm * t)
{
    #define MONTHS_PER_YEAR 12
    static const int cumdays[MONTHS_PER_YEAR] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };

    long year;
    time_t result;

    year = 1900 + t->tm_year + t->tm_mon / MONTHS_PER_YEAR;
    result = (year - 1970) * 365 + cumdays[t->tm_mon % MONTHS_PER_YEAR];
    result += (year - 1968) / 4;
    result -= (year - 1900) / 100;
    result += (year - 1600) / 400;
    if ((year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0) &&
        (t->tm_mon % MONTHS_PER_YEAR) < 2)
        result--;

    result += t->tm_mday - 1;
    result *= 24;

    result += t->tm_hour;
    result *= 60;

    result += t->tm_min;
    result *= 60;

    result += t->tm_sec;
    if (t->tm_isdst == 1)
        result -= 3600;

    return (result);
}

void gp_time_to_mav(gp_can_mav_get_rsp_t *rsp, const struct tm *ti)
{
    time_t t = timegm(ti);
    rsp->mav.value[0] = t & 0xff;
    rsp->mav.value[1] = (t >> 8) & 0xff;
    rsp->mav.value[2] = (t >> 16) & 0xff;
    rsp->mav.value[3] = (t >> 24) & 0xff;
}

time_t gp_time_from_mav(const gp_can_mav_set_req_t* request)
{
    return ((uint32_t)request->mav.value[3] << 24) |
           ((uint32_t)request->mav.value[2] << 16) |
           ((uint32_t)request->mav.value[1] << 8) |
            (uint32_t)request->mav.value[0];
}

void gp_set_bp_detect_asserted_out(bool assert) {
    if (assert) {
        GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
    } else {
        GpioDataRegs.GPASET.bit.GPIO28 = 1;
    }
}

bool gp_get_bp_detect_asserted_out(void) {
    return !GpioDataRegs.GPADAT.bit.GPIO28;
}

void gp_set_intr_asserted_out(bool assert) {
    if (assert) {
        GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
#if defined(PIN29_DEBUG_GP_INTR)
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
#endif
    } else {
        GpioDataRegs.GPASET.bit.GPIO26 = 1;
#if defined(PIN29_DEBUG_GP_INTR)
        GpioDataRegs.GPASET.bit.GPIO29 = 1;
#endif
    }
}

bool gp_get_intr_asserted_out(void) {
    return !GpioDataRegs.GPADAT.bit.GPIO26;
}

void gp_set_pwron_asserted_out(bool assert) {
    if (assert) {
        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
    } else {
        GpioDataRegs.GPASET.bit.GPIO22 = 1;
    }
}

bool gp_get_pwron_asserted_out(void) {
    return !GpioDataRegs.GPADAT.bit.GPIO22;
}

void gp_set_charging_asserted_out(bool assert) {
    if (assert) {
        GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
    } else {
        GpioDataRegs.GPASET.bit.GPIO23 = 1;
    }
}

bool gp_get_charging_asserted_out(void) {
    return !GpioDataRegs.GPADAT.bit.GPIO23;
}

bool gp_get_von_asserted_in(void) {
    return GpioDataRegs.GPADAT.bit.GPIO6;
}
