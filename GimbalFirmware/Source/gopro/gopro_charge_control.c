/*
 * gopro_charge_control.c
 *
 *  Created on: Apr 8, 2015
 *      Author: abamberger
 */

#include "PeripheralHeaderIncludes.h"
#include "gopro/gopro_charge_control.h"
#include "gopro/gopro_interface.h"
#include "can/cand_BitFields.h"
#include "can/cand.h"

static void send_charge_event(GoProChargeControlEvent charge_event);

GoProChargeState charge_state = GOPRO_CHARGING;

void gp_update_charge_control(int16 proc_temperature, int16 batt_level)
{
    switch (charge_state) {
        case GOPRO_CHARGING:
            // If we're currently charging, need to check temperature and battery level limits
            if (proc_temperature > HALT_CHARGING_TEMP_LIMIT_C) {
                //gp_disable_charging();
                gp_enable_charging();
                send_charge_event(CHARGING_HALTED_OVER_TEMP);
                charge_state = GOPRO_CHARGE_HALTED_OVER_TEMP;
            } else if (batt_level > HALT_CHARGING_CHARGE_LEVEL) {
                //gp_disable_charging();
                gp_enable_charging();
                send_charge_event(CHARGING_HALTED_CAPACITY_THRESHOLD_REACHED);
                charge_state = GOPRO_CHARGE_HALTED_OVER_CAPACITY;
            }
            break;

        case GOPRO_CHARGE_HALTED_OVER_TEMP:
            // See if we've cooled down enough to resume charging again
            if (proc_temperature <= RESUME_CHARGING_TEMP_LIMIT_C) {
                // If we've cooled down enough, check capacity to make sure we shouldn't still be disabled
                if (batt_level <= RESUME_CHARGING_CHARGE_LEVEL) {
                    gp_enable_charging();
                    send_charge_event(CHARGING_RESUMED_UNDER_TEMP);
                    charge_state = GOPRO_CHARGING;
                } else {
                    send_charge_event(CHARGING_HALTED_CAPACITY_THRESHOLD_REACHED);
                    charge_state = GOPRO_CHARGE_HALTED_OVER_CAPACITY;
                }
            }
            break;

        case GOPRO_CHARGE_HALTED_OVER_CAPACITY:
            // See if battery charge level has reduced enough to resume charging again
            if (batt_level <= RESUME_CHARGING_CHARGE_LEVEL) {
                // If we're at a low enough battery level, check temperature to make sure we shouldn't still be disabled
                if (proc_temperature <= RESUME_CHARGING_TEMP_LIMIT_C) {
                    gp_enable_charging();
                    send_charge_event(CHARGING_RESUMED_UNDER_CAPACITY_THRESHOLD);
                    charge_state = GOPRO_CHARGING;
                } else {
                    send_charge_event(CHARGING_HALTED_OVER_TEMP);
                    charge_state = GOPRO_CHARGE_HALTED_OVER_TEMP;
                }
            }
            break;
    }
}

static void send_charge_event(GoProChargeControlEvent charge_event)
{
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_GP_CHARGE_CONTROL_EVENT, (Uint8*)(&charge_event), 1);
}
