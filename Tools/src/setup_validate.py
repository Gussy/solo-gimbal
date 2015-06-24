import setup_comutation
import setup_factory
import setup_param
from distutils.version import LooseVersion
from pymavlink.mavparm import MAVParmDict
import setup_mavlink

EXPECTED_VERSION = '0.16.0'
EXPECTED_BROADCAST = 0

EXPECTED_SERIAL_NUMBER_START = 'GB11A'
EXPETED_ASSEMBLY_DATE_MIN = 1434778800 # Sat Jun 20 02:40:00 BRT 2015

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
EXPECTED_PITCH_P = 3.00
EXPECTED_PITCH_I = 0.50
EXPECTED_PITCH_D = 0.10
EXPECTED_ROLL_P = 5.00
EXPECTED_ROLL_I = 0.50
EXPECTED_ROLL_D = 0.10
EXPECTED_YAW_P = 0.50
EXPECTED_YAW_I = 0.00
EXPECTED_YAW_D = 7.00
EXPECTED_K_RATE = 10.0

class Results:
    Pass, Fail, Error = 'pass', 'fail', 'error'

def show(link):
    ver = setup_factory.readSWver(link)
    asm_date = setup_factory.get_assembly_time(link)
    serial = setup_factory.get_serial_number(link)
    pitch_com, roll_com, yaw_com = setup_comutation.getAxisCalibrationParams(link)
    joint = setup_param.get_offsets(link, 'JNT')
    gyro = setup_param.get_offsets(link, 'GYRO')
    acc = setup_param.get_offsets(link, 'ACC')
    k_rate = setup_param.fetch_param(link, "GMB_K_RATE")

    if ver != None and pitch_com != None and roll_com != None and yaw_com != None and joint and gyro and acc and k_rate != None:
        params = {
            'version': ver,
            'pitch': {
                'icept': pitch_com[0],
                'slope': pitch_com[1],
            },
            'roll': {
                'icept': roll_com[0],
                'slope': roll_com[1],
            },
            'yaw': {
                'icept': yaw_com[0],
                'slope': yaw_com[1],
            },
            'joint': {
                'x': joint.x,
                'y': joint.y,
                'z': joint.z,
            },
            'gyro': {
                'x': gyro.x,
                'y': gyro.y,
                'z': gyro.z,
            },
            'accel': {
                'x': acc.x,
                'y': acc.y,
                'z': acc.z,
            },
            'k_rate': k_rate.param_value
        }
        return params
    else:
        return None

def validate_version(link):
    swver = readSWver(link)
    if not swver:
        return Results.Error
    ver = LooseVersion("%i.%i.%i" % (swver[0], swver[1], swver[2]))
    ver_expected = LooseVersion(EXPECTED_VERSION)
    if ver >= ver_expected:
        return Results.Pass
    else:
        return Results.Fail


def validate_comutation_axis(link, axis, i_max, i_min, s_max, s_min):
    icept = axis[0]
    slope = axis[1]
    if (icept == 0) or (i_min >= icept) or (i_max <= icept):
        return False
    elif (icept == 0) or (s_min >= slope) or (s_max <= slope):
        return False
    else:
        return True

def validate_comutation(link):
    "The acceptable range values where collected from the batch of DVT1 gimbals"
    pitch_com, roll_com, yaw_com = setup_comutation.getAxisCalibrationParams(link)
    if pitch_com == None or roll_com == None or yaw_com == None:
        return Results.Error
    if (validate_comutation_axis(link, pitch_com, EXPECTED_PITCH_ICEPT_MAX, EXPECTED_PITCH_ICEPT_MIN, EXPECTED_PITCH_SLOPE_MAX, EXPECTED_PITCH_SLOPE_MIN) and 
        validate_comutation_axis(link, roll_com, EXPECTED_ROLL_ICEPT_MAX, EXPECTED_ROLL_ICEPT_MIN, EXPECTED_ROLL_SLOPE_MAX, EXPECTED_ROLL_SLOPE_MIN) and 
        validate_comutation_axis(link, yaw_com, EXPECTED_YAW_ICEPT_MAX, EXPECTED_YAW_ICEPT_MIN, EXPECTED_YAW_SLOPE_MAX, EXPECTED_YAW_SLOPE_MIN)):
        return Results.Pass
    else:
        return Results.Fail

def validate_joints(link):
    "The acceptable range values where collected from the batch of DVT1 gimbals"
    joint = setup_param.get_offsets(link, 'JNT')
    if not joint:
        return Results.Error
    if ((joint.x <= EXPECTED_JOINT_X_MAX) and (joint.x >= EXPECTED_JOINT_X_MIN) and (joint.x != 0) and
        (joint.y <= EXPECTED_JOINT_Y_MAX) and (joint.y >= EXPECTED_JOINT_Y_MIN) and (joint.y != 0) and
        (joint.z <= EXPECTED_JOINT_Z_MAX) and (joint.z >= EXPECTED_JOINT_Z_MIN) and (joint.z != 0)):
        return Results.Pass
    else:
        return Results.Fail

def validate_gyros(link):
    "The acceptable range values where collected from the batch of DVT1 gimbals"
    gyro = setup_param.get_offsets(link, 'GYRO')
    if not gyro:
        return Results.Error
    if ((gyro.x <= EXPECTED_GYRO) and (gyro.x >= -EXPECTED_GYRO) and (gyro.x != 0) and
        (gyro.y <= EXPECTED_GYRO) and (gyro.y >= -EXPECTED_GYRO) and (gyro.y != 0) and
        (gyro.z <= EXPECTED_GYRO) and (gyro.z >= -EXPECTED_GYRO) and (gyro.z != 0)):
        return Results.Pass
    else:
        return Results.Fail

def validate_accelerometers(link):
    "Since there is no accelerometer cal yet, just check if the values are zeroed"
    acc = setup_param.get_offsets(link, 'ACC')
    if not acc:
        return Results.Error
    if acc.x == 0 and acc.y == 0 and acc.z == 0:
        return Results.Pass
    else:
        return Results.Fail

def validate_gain_axis(link, axis, p_e, i_e, d_e):
    p, i, d = setup_param.get_gains(link, axis)
    if p == None or i == None or d == None:
        return None
    return (abs(p - p_e) < GAIN_TOLERANCE and
            abs(i - i_e) < GAIN_TOLERANCE and
            abs(d - d_e) < GAIN_TOLERANCE)

def validate_k_rate(link, value):
    k_rate = setup_param.fetch_param(link, "GMB_K_RATE")
    if k_rate:
        return (abs(k_rate.param_value - value) < GAIN_TOLERANCE)
    else:
        return None

def validate_gains(link):
    pitch = validate_gain_axis(link, 'PITCH', EXPECTED_PITCH_P, EXPECTED_PITCH_I, EXPECTED_PITCH_D)
    roll = validate_gain_axis(link, 'ROLL',  EXPECTED_ROLL_P, EXPECTED_ROLL_I, EXPECTED_ROLL_D)
    yaw = validate_gain_axis(link, 'YAW',   EXPECTED_YAW_P, EXPECTED_YAW_I, EXPECTED_YAW_D)
    krate = validate_k_rate(link, EXPECTED_K_RATE)
    if pitch == True and roll == True and yaw == True and krate == True:
        return Results.Pass
    elif pitch == False or roll == False or yaw == False or krate == False:
        return Results.Fail
    else:
        return Results.Error


def validate_date(link):
    assembly_time = setup_factory.get_assembly_time(link)
    if (assembly_time > EXPETED_ASSEMBLY_DATE_MIN):
        print 'Assembly date\t- PASS'
    else:
        print 'Assembly date\t- FAIL - assembly date was not set on the factory (--date)'


def validate_serial_number(link):
    serial_number = setup_factory.get_serial_number(link)
    if (serial_number) and (serial_number.startswith(EXPECTED_SERIAL_NUMBER_START)):
        print 'Serial number\t- PASS'
    else:
        print 'Serial number\t- FAIL - Serial number was not set (--serialnumber SERIALNUMBER)'
        
def validate(link):
    validation = {
        'version': validate_version(link),
        'serial': validate_serial_number(link),
        'date': validate_date(link),
        'commutation': validate_comutation(link),
        'joints': validate_joints(link),
        'gyros': validate_gyros(link),
        'accels': validate_accelerometers(link),
        'gains': validate_gains(link)
    }
    return validation

def restore_defaults(link):
    parameters = MAVParmDict()
    parameters.mavset(link.file, "GMB_PITCH_P", EXPECTED_PITCH_P);
    parameters.mavset(link.file, "GMB_PITCH_I", EXPECTED_PITCH_I);
    parameters.mavset(link.file, "GMB_PITCH_D", EXPECTED_PITCH_D);
    parameters.mavset(link.file, "GMB_ROLL_P", EXPECTED_ROLL_P);
    parameters.mavset(link.file, "GMB_ROLL_I", EXPECTED_ROLL_I);
    parameters.mavset(link.file, "GMB_ROLL_D", EXPECTED_ROLL_D);
    parameters.mavset(link.file, "GMB_YAW_P", EXPECTED_YAW_P);
    parameters.mavset(link.file, "GMB_YAW_I", EXPECTED_YAW_I);
    parameters.mavset(link.file, "GMB_YAW_D", EXPECTED_YAW_D);
    parameters.mavset(link.file, "GMB_K_RATE", EXPECTED_K_RATE);
    parameters.mavset(link.file, "GMB_BROADCAST", EXPECTED_BROADCAST);    
    setup_param.commit_to_flash(link)
    setup_mavlink.reset_gimbal(link)
    return True
