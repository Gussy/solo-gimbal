import setup_comutation
from setup_read_sw_version import readSWver
import setup_param

def show(link):
    ver = readSWver(link)
    pitch_com, roll_com, yaw_com = setup_comutation.getAxisCalibrationParams(link)
    joint = setup_param.get_offsets(link, 'JNT')
    gyro = setup_param.get_offsets(link, 'GYRO')
    acc = setup_param.get_offsets(link, 'ACC')
    k_rate = setup_param.fetch_param(link, "GMB_K_RATE").param_value
    print "sw_ver, pitch_icept, pitch_slope, roll_icept, roll_slope, yaw_icept, yaw_slope, joint_z, joint_y, joint_z, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, k_rate"
    print "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s" % (ver, pitch_com[1], pitch_com[2], roll_com[1], roll_com[2], yaw_com[1], yaw_com[2], joint.x, joint.y, joint.z, gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, k_rate)