#include "gopro/gopro_helpers.h"
#include "can/cand.h"
#include "PeripheralHeaderIncludes.h"

bool gp_send_mav_get_response(uint8_t cmd_id, uint8_t value) {
    cand_tx_response(CAND_ID_AZ, CAND_PID_GOPRO_GET_RESPONSE, (((uint32_t)cmd_id) << 8) | (((uint32_t)value) << 0));
    return true;
}

bool gp_send_mav_set_response(uint8_t cmd_id, uint8_t result) {
    cand_tx_response(CAND_ID_AZ, CAND_PID_GOPRO_SET_RESPONSE, (((uint32_t)cmd_id) << 8) | (((uint32_t)result) << 0));
    return true;
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
