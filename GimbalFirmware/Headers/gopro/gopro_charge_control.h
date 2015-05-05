/*
 * gopro_charge_control.h
 *
 *  Created on: Apr 8, 2015
 *      Author: abamberger
 */

#ifndef GOPRO_CHARGE_CONTROL_H_
#define GOPRO_CHARGE_CONTROL_H_

#define HALT_CHARGING_TEMP_LIMIT_C 60
#define RESUME_CHARGING_TEMP_LIMIT_C 58
#define HALT_CHARGING_CHARGE_LEVEL 50
#define RESUME_CHARGING_CHARGE_LEVEL 45

typedef enum {
    CHARGING_HALTED_OVER_TEMP,
    CHARGING_RESUMED_UNDER_TEMP,
    CHARGING_HALTED_CAPACITY_THRESHOLD_REACHED,
    CHARGING_RESUMED_UNDER_CAPACITY_THRESHOLD
} GoProChargeControlEvent;

typedef enum {
    GOPRO_CHARGING,
    GOPRO_CHARGE_HALTED_OVER_TEMP,
    GOPRO_CHARGE_HALTED_OVER_CAPACITY
} GoProChargeState;

void gp_update_charge_control(int16 proc_temperature, int16 batt_level);

#endif /* GOPRO_CHARGE_CONTROL_H_ */
