#ifndef GYRO_KINEMATICS_CORRECTION_H_
#define GYRO_KINEMATICS_CORRECTION_H_

#include "PeripheralHeaderIncludes.h"

#define M_PI (float)3.14159265358979323846

extern float sin_phi;
extern float cos_phi;
extern float sin_theta;
extern float cos_theta;

void update_joint_ang_trig();
void transform_ang_vel_to_joint_rate(int16* gyro_in, int16* gyro_out);

#endif /* GYRO_KINEMATICS_CORRECTION_H_ */
