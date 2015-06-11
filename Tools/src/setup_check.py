import setup_comutation
from setup_read_sw_version import readSWver
import setup_param
from distutils.version import LooseVersion

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
    ver_expected = LooseVersion('0.15.2')
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
    if (validate_comutation_axis(link, pitch_com, 0.30, 0.10, 0.14, 0.12) and 
        validate_comutation_axis(link, roll_com,  0.51, 0.35, 0.14, 0.11) and 
        validate_comutation_axis(link,yaw_com,    0.54, 0.40, 0.12, 0.10)):
        print 'Comutation \t- PASS'
    else:
        print 'Comutation \t- FAIL - please redo the comutation calibration (-c)'


def validate_joints(link):
    "The acceptable range values where collected from the batch of DVT1 gimbals"
    joint = setup_param.get_offsets(link, 'JNT')
    if ((joint.x <= 0.12) and (joint.x >= -0.02) and (joint.x != 0) and
        (joint.y <= 0.15) and (joint.y >= -0.07) and (joint.y != 0) and
        (joint.z <= 0.18) and (joint.z >= -0.18) and (joint.z != 0)):
        print 'Joints  \t- PASS'
    else:
        print 'Joints  \t- FAIL - redo joint calibration (-j)'

def validate_gyros(link):
    "The acceptable range values where collected from the batch of DVT1 gimbals"
    gyro = setup_param.get_offsets(link, 'GYRO')
    if ((gyro.x <= 5E-04) and (gyro.x >= -5E-04) and (gyro.x != 0) and
        (gyro.y <= 5E-04) and (gyro.y >= -5E-04) and (gyro.y != 0) and
        (gyro.z <= 5E-04) and (gyro.z >= -5E-04) and (gyro.z != 0)):
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

def validate(link):
    validate_version(link)
    validate_comutation(link)
    validate_joints(link)
    validate_gyros(link)
    validate_accelerometers(link)
