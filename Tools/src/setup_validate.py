import setup_comutation
from setup_read_sw_version import readSWver
import setup_param
from distutils.version import LooseVersion

EXPECTED_VERSION = '0.15.2'

EXPECTED_PITCH_ICEPT_MAX = 0.30
EXPECTED_PITCH_ICEPT_MIN = 0.10
EXPECTED_PITCH_SLOPE_MAX = 0.14
EXPECTED_PITCH_SLOPE_MIN = 0.12
EXPECTED_ROLL_ICEPT_MIN = 0.35
EXPECTED_ROLL_ICEPT_MAX = 0.51
EXPECTED_ROLL_SLOPE_MAX = 0.14
EXPECTED_ROLL_SLOPE_MIN = 0.11
EXPECTED_YAW_ICEPT_MAX = 0.54
EXPECTED_YAW_ICEPT_MIN = 0.40
EXPECTED_YAW_SLOPE_MAX = 0.12
EXPECTED_YAW_SLOPE_MIN = 0.10

EXPECTED_JOINT_X_MAX = 0.12
EXPECTED_JOINT_X_MIN = -0.02
EXPECTED_JOINT_Y_MAX = 0.15
EXPECTED_JOINT_Y_MIN = -0.07
EXPECTED_JOINT_Z_MAX = 0.18
EXPECTED_JOINT_Z_MIN = -0.18

EXPECTED_GYRO = 5E-04

GAIN_TOLERANCE = 1e-6
EXPECTED_PITCH_P = 3.50
EXPECTED_PITCH_I = 0.25
EXPECTED_PITCH_D = 0.10
EXPECTED_ROLL_P = 4.00
EXPECTED_ROLL_I = 0.75
EXPECTED_ROLL_D = 1.00
EXPECTED_YAW_P = 4.00
EXPECTED_YAW_I = 1.00
EXPECTED_YAW_D = 0.10
EXPECTED_K_RATE = 10.0

def show(link):
    ver = readSWver(link)
    pitch_com, roll_com, yaw_com = setup_comutation.getAxisCalibrationParams(link)
    joint = setup_param.get_offsets(link, 'JNT')
    gyro = setup_param.get_offsets(link, 'GYRO')
    acc = setup_param.get_offsets(link, 'ACC')
    k_rate = setup_param.fetch_param(link, "GMB_K_RATE").param_value
    print "sw_ver, pitch_icept, pitch_slope, roll_icept, roll_slope, yaw_icept, yaw_slope, joint_z, joint_y, joint_z, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, k_rate"
    print "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s" % (ver, pitch_com[1], pitch_com[2], roll_com[1], roll_com[2], yaw_com[1], yaw_com[2], joint.x, joint.y, joint.z, gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, k_rate)
    

def validate_version(link):
    ver = LooseVersion(readSWver(link))
    ver_expected = LooseVersion(EXPECTED_VERSION)
    if ver >= ver_expected:
        print 'Version \t- PASS'
    else:
        print 'Version \t- FAIL - please update with software ' + str(ver_expected) + ' or newer'


def validate_comutation_axis(link, axis, i_max, i_min, s_max, s_min):
    icept = axis[1]
    slope = axis[2]
    if (icept == 0) or (i_min >= icept) or (i_max <= icept):
        return False
    elif (icept == 0) or (s_min >= slope) or (s_max <= slope):
        return False
    else:
        return True

def validate_comutation(link):
    "The acceptable range values where collected from the batch of DVT1 gimbals"
    pitch_com, roll_com, yaw_com = setup_comutation.getAxisCalibrationParams(link)
    if (validate_comutation_axis(link, pitch_com, EXPECTED_PITCH_ICEPT_MAX, EXPECTED_PITCH_ICEPT_MIN, EXPECTED_PITCH_SLOPE_MAX, EXPECTED_PITCH_SLOPE_MIN) and 
        validate_comutation_axis(link, roll_com, EXPECTED_ROLL_ICEPT_MAX, EXPECTED_ROLL_ICEPT_MIN, EXPECTED_ROLL_SLOPE_MAX, EXPECTED_ROLL_SLOPE_MIN) and 
        validate_comutation_axis(link, yaw_com, EXPECTED_YAW_ICEPT_MAX, EXPECTED_YAW_ICEPT_MIN, EXPECTED_YAW_SLOPE_MAX, EXPECTED_YAW_SLOPE_MIN)):
        print 'Comutation \t- PASS'
    else:
        print 'Comutation \t- FAIL - please redo the comutation calibration (-c)'


def validate_joints(link):
    "The acceptable range values where collected from the batch of DVT1 gimbals"
    joint = setup_param.get_offsets(link, 'JNT')
    if ((joint.x <= EXPECTED_JOINT_X_MAX) and (joint.x >= EXPECTED_JOINT_X_MIN) and (joint.x != 0) and
        (joint.y <= EXPECTED_JOINT_Y_MAX) and (joint.y >= EXPECTED_JOINT_Y_MIN) and (joint.y != 0) and
        (joint.z <= EXPECTED_JOINT_Z_MAX) and (joint.z >= EXPECTED_JOINT_Z_MIN) and (joint.z != 0)):
        print 'Joints  \t- PASS'
    else:
        print 'Joints  \t- FAIL - redo joint calibration (-j)'

def validate_gyros(link):
    "The acceptable range values where collected from the batch of DVT1 gimbals"
    gyro = setup_param.get_offsets(link, 'GYRO')
    if ((gyro.x <= EXPECTED_GYRO) and (gyro.x >= -EXPECTED_GYRO) and (gyro.x != 0) and
        (gyro.y <= EXPECTED_GYRO) and (gyro.y >= -EXPECTED_GYRO) and (gyro.y != 0) and
        (gyro.z <= EXPECTED_GYRO) and (gyro.z >= -EXPECTED_GYRO) and (gyro.z != 0)):
        print 'Gyros   \t- PASS'
    else:
        print 'Gyros   \t- FAIL - redo gyro calibration (-g)'

def validate_accelerometers(link):
    "Since there is no accelerometer cal yet, just check if the values are zeroed"
    acc = setup_param.get_offsets(link, 'ACC')
    if ((acc.x == 0) and
        (acc.y == 0) and
        (acc.z == 0)):
        print 'Accelerometer\t- PASS'
    else:
        print 'Accelerometer\t- FAIL - redo accelerometer calibration (-a)'

def validate_gain_axis(link,axis,p_e,i_e,d_e):
    p, i, d = setup_param.get_gains(link, axis)
    return (abs(p - p_e) < GAIN_TOLERANCE and
            abs(i - i_e) < GAIN_TOLERANCE and
            abs(d - d_e) < GAIN_TOLERANCE)

def validate_k_rate(link, value):
    k_rate = setup_param.fetch_param(link, "GMB_K_RATE").param_value
    return (abs(k_rate - value) < GAIN_TOLERANCE)

def validate_gains(link):
    if (validate_gain_axis(link, 'PITCH', EXPECTED_PITCH_P, EXPECTED_PITCH_I, EXPECTED_PITCH_D) and
        validate_gain_axis(link, 'ROLL',  EXPECTED_ROLL_P, EXPECTED_ROLL_I, EXPECTED_ROLL_D) and
        validate_gain_axis(link, 'YAW',   EXPECTED_YAW_P, EXPECTED_YAW_I, EXPECTED_YAW_D) and
        validate_k_rate(link,EXPECTED_K_RATE)):
        print 'Gains   \t- PASS'
    else:
        print 'Gains   \t- FAIL'

def validate(link):
    validate_version(link)
    validate_comutation(link)
    validate_joints(link)
    validate_gyros(link)
    validate_accelerometers(link)
    validate_gains(link)