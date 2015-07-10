#include "hardware/HWSpecific.h"
#include "control/gyro_kinematics_correction.h"
#include "hardware/encoder.h"

#include <math.h>

static float phi = 0;
static float theta = 0;

int do_gyro_correction(int16* gyro_in, int16* encoder_in, int16* gyro_out)
{
    theta = ((2.0 * M_PI * encoder_in[EL]) / (1.0 * ENCODER_COUNTS_PER_REV));
    phi = ((2.0 * M_PI * encoder_in[ROLL]) / (1.0 * ENCODER_COUNTS_PER_REV));

    gyro_out[ROLL]  =                 cos(theta)*gyro_in[ROLL] +               +               sin(theta)*gyro_in[AZ];
    gyro_out[EL]    =        sin(theta)*tan(phi)*gyro_in[ROLL] + 1*gyro_in[EL] + (-1)*cos(theta)*tan(phi)*gyro_in[AZ];
    gyro_out[AZ]    = (-1)*(sin(theta)/cos(phi))*gyro_in[ROLL] +               +    (cos(theta)/cos(phi))*gyro_in[AZ];

    return 0;
}
