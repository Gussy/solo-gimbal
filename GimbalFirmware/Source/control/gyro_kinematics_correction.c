#include "hardware/HWSpecific.h"
#include "control/gyro_kinematics_correction.h"
#include "hardware/encoder.h"

#include <math.h>

int do_gyro_correction(int16* gyro_in, int16* encoder_in, int16* gyro_out)
{
    float theta = ((2.0 * M_PI * encoder_in[EL]) / (1.0 * ENCODER_COUNTS_PER_REV));
    float phi = ((2.0 * M_PI * encoder_in[ROLL]) / (1.0 * ENCODER_COUNTS_PER_REV));

    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    float sin_phi = sinf(phi);
    float cos_phi = cosf(phi);
    float sec_phi = 1.0f/cos_phi;
    float tan_phi = sin_phi/cos_phi;

    gyro_out[ROLL] = gyro_in[ROLL]*cos_theta+gyro_in[AZ]*sin_theta;
    gyro_out[EL]   = gyro_in[ROLL]*sin_theta*tan_phi-gyro_in[AZ]*cos_theta*tan_phi+gyro_in[EL];
    gyro_out[AZ]   = sec_phi*(gyro_in[AZ]*cos_theta-gyro_in[ROLL]*sin_theta);

    return 0;
}
