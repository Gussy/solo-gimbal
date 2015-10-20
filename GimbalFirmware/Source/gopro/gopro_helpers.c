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
