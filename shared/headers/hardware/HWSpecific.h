#ifndef HWSPECIFIC_H_
#define HWSPECIFIC_H_

typedef enum {
    EL = 0,
    AZ = 1,
    ROLL = 2,
    AXIS_CNT
} GimbalAxis;

// The defines below are not used in the bootlader project
// XXX: These defines should really split out into seperate files
#ifndef BOOTLOADER
typedef enum {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS
} GyroAxis;

extern float AxisCalibrationSlopes[AXIS_CNT];
extern float AxisCalibrationIntercepts[AXIS_CNT];
extern float AxisTorqueLoopKp[AXIS_CNT];
extern float AxisTorqueLoopKi[AXIS_CNT];
extern float AxisTorqueLoopKd[AXIS_CNT];

#endif

#endif /* HWSPECIFIC_H_ */
