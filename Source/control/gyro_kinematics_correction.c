/*
 * gyro_kinematics_correction.c
 *
 *  Created on: Sep 12, 2014
 *      Author: abamberger
 */

#include "hardware/HWSpecific.h"
#include "control/gyro_kinematics_correction.h"

#include <math.h>

//static float theta_rate = 0;
//static float theta_old = 0;
static float phi = 0;
static float theta = 0;

int do_gyro_correction(int16* gyro_in, int16* encoder_in, int16* gyro_out)
{
    theta = ((2.0 * M_PI * encoder_in[EL]) / (1.0 * ENCODER_COUNTS_PER_REV));
    phi = ((2.0 * M_PI * encoder_in[ROLL]) / (1.0 * ENCODER_COUNTS_PER_REV));

    //theta_rate = 100*1689.7212928*second_order_filter(theta,fv,1833.046411055, -1833.046411055, 0, -0.222030941, 0);

    gyro_out[ROLL] = gyro_in[ROLL] * cos(theta) + gyro_in[AZ] * sin(theta);
    gyro_out[EL] = gyro_in[EL];
    gyro_out[AZ] = -1 * sin(theta) * cos(phi) * gyro_in[ROLL] + sin(phi) * gyro_in[EL] + cos(phi) * cos(theta) * gyro_in[AZ]; //-1*sin(phi)*theta_rate;

    return 0;
}
