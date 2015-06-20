"""
"""
import math

from setup_mavlink import get_current_joint_angles, get_current_delta_angles, get_current_delta_velocity
from setup_param import set_offsets, message_brodcasting
from pymavlink.rotmat import Vector3


NUMBER_OF_SAMPLES = 200
ENCODER_COUNTS_PER_RADIAN = 1000.0/(2*math.pi)

def getAverage(link,get_variable, sample_count =  NUMBER_OF_SAMPLES):
    sum_angles = Vector3()
    for i in range(sample_count):
        angles = get_variable(link)
        sum_angles += angles    
    offset = sum_angles/sample_count
    return offset

def calibrate_joints(link):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_joint_angles)
    print [average.x,average.y,average.z]
    set_offsets(link,'JNT',average)
    message_brodcasting(link, False)
    return

def calibrate_gyro(link):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_delta_angles)
    print [average.x,average.y,average.z]
    set_offsets(link,'GYRO',average)
    message_brodcasting(link, False)
    return


def getAccelSample(link, ui_msg, AVG_COUNT=10):
    raw_input(ui_msg)
    v = getAverage(link, get_current_delta_velocity, sample_count=AVG_COUNT)
    v = v*100 # sample needs to be in units of m/s
    return np.asarray([v.x,v.y,v.z])

def calibrate_accel(link):
    message_brodcasting(link, True)
        
    # Get samples
    samples = [];
    samples.append(getAccelSample(link,"Place the gimbal leveled"))
    samples.append(getAccelSample(link,"Place the gimbal on it's side"))
    samples.append(getAccelSample(link,"Place the gimbal on it's back"))
    samples.append(getAccelSample(link,"Place the gimbal on it's other side"))
    samples.append(getAccelSample(link,"Place the gimbal on it's front side"))
    samples.append(getAccelSample(link,"Place the gimbal upside-down"))
    print ' '
    
    
    # Apply Math
    for vector in samples:
        print vector
        
    # Save values
    set_offsets(link, 'ACC', Vector3()) # TODO: implement accel calibration
    message_brodcasting(link, False)
    return


