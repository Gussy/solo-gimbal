#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "PM_Sensorless-Settings.h"
#include "hardware/device_init.h"
#include "hardware/encoder.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "parameters/flash_params.h"

static const int EncoderSignMap[AXIS_CNT] = {
        1, // EL
        -1, // AZ
        -1  // ROLL
};

void UpdateEncoderReadings(EncoderParms* encoder_parms, ControlBoardParms* cb_parms)
{
    encoder_parms->raw_theta = AdcResult.ADCRESULT5;
    if (encoder_parms->raw_theta < 0) {
        encoder_parms->raw_theta += ANALOG_POT_MECH_DIVIDER;
    } else if (encoder_parms->raw_theta > ANALOG_POT_MECH_DIVIDER) {
        encoder_parms->raw_theta -= ANALOG_POT_MECH_DIVIDER;
    }

    // AZ axis motor is mounted opposite of the encoder relative to the other two axes, so we need to invert it here if we're AZ
    // On new hardware, EL is also flipped relative to what it was on the old hardware
    if ((GetBoardHWID() == AZ) || (GetBoardHWID() == EL)) {
        encoder_parms->raw_theta = ANALOG_POT_MECH_DIVIDER - encoder_parms->raw_theta;
    }

    encoder_parms->mech_theta = ((float)encoder_parms->raw_theta) / ANALOG_POT_MECH_DIVIDER;
    encoder_parms->corrected_mech_theta = (encoder_parms->mech_theta - encoder_parms->calibration_intercept) / encoder_parms->calibration_slope;
    encoder_parms->elec_theta = encoder_parms->corrected_mech_theta - floor(encoder_parms->corrected_mech_theta);

    // Calculate the emulated encoder value to communicate back to the control board
    encoder_parms->virtual_counts = (encoder_parms->mech_theta * ((float)ENCODER_COUNTS_PER_REV)) -5000;

    // Invert the encoder reading if necessary to make sure it counts up in the right direction
    // This is necessary for the kinematics math to work properly
    if (EncoderSignMap[GetBoardHWID()] < 0) {
        encoder_parms->virtual_counts = ENCODER_COUNTS_PER_REV - encoder_parms->virtual_counts;
    }

    // Convert the virtual counts to be symmetric around 0
    if (encoder_parms->virtual_counts < -(ENCODER_COUNTS_PER_REV / 2)) {
        encoder_parms->virtual_counts += ENCODER_COUNTS_PER_REV;
    } else if (encoder_parms->virtual_counts >= (ENCODER_COUNTS_PER_REV / 2)) {
        encoder_parms->virtual_counts -= ENCODER_COUNTS_PER_REV;
    }

    // Apply joing angle offsets
    encoder_parms->virtual_counts -= getAxisJointOffset((GimbalAxis)GetBoardHWID());

    // Accumulate the virtual counts at the torque loop rate (10kHz), which will then be averaged to be sent out
    // at the rate loop rate (1kHz)
    encoder_parms->virtual_counts_accumulator += encoder_parms->virtual_counts;
    encoder_parms->virtual_counts_accumulated++;

    // We've received our own encoder value, so indicate as such
    if (!cb_parms->encoder_value_received[EL]) {
        cb_parms->encoder_value_received[EL] = TRUE;
    }
}

int16 getAxisJointOffset(GimbalAxis axis)
{
	switch (axis) {
	case ROLL:
		return RAD_TO_ENCODER_FORMAT(flash_params.offset_joint[X_AXIS]);
	case EL:
		return RAD_TO_ENCODER_FORMAT(flash_params.offset_joint[Y_AXIS]);
	case AZ:
		return RAD_TO_ENCODER_FORMAT(flash_params.offset_joint[Z_AXIS]);
	default:
		return 0;
	}
}

/**
 * Is axis near from top mech hardstop
 * This lose definition allows us to not have to compensate for the home position offsets
 */
int nearHardStopTop(EncoderParms* encoder_parms) {
	switch (GetBoardHWID()) {
	default:
	case AZ:
		return encoder_parms->mech_theta
				> (0.5 + RADIANS_TO_REV(ANGLE_MAX_AZ - ANGLE_TOLERANCE_AZ));
	case ROLL:
		return encoder_parms->mech_theta
				> (0.5 + RADIANS_TO_REV(ANGLE_MAX_ROLL - ANGLE_TOLERANCE_ROLL));
	case EL:
		return encoder_parms->mech_theta
				> (0.5 + RADIANS_TO_REV(ANGLE_MAX_EL - ANGLE_TOLERANCE_EL));
	}
}

/**
 * Is axis near from bottom mech hardstop
 * This lose definition allows us to not have to compensate for the home position offsets
 */
int nearHardStopBottom(EncoderParms* encoder_parms) {
	switch (GetBoardHWID()) {
	default:
	case AZ:
		return encoder_parms->mech_theta
				< (0.5 + RADIANS_TO_REV(ANGLE_MIN_AZ + ANGLE_TOLERANCE_AZ));
	case ROLL:
		return encoder_parms->mech_theta
				< (0.5 + RADIANS_TO_REV(ANGLE_MIN_ROLL + ANGLE_TOLERANCE_ROLL));
	case EL:
		return encoder_parms->mech_theta
				< (0.5 + RADIANS_TO_REV(ANGLE_MIN_EL + ANGLE_TOLERANCE_EL));
	}
}
