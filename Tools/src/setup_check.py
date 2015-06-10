import setup_comutation
from setup_read_sw_version import readSWver
import setup_param

def setup_check(link):
    ver = readSWver(link)
    pitch_com, roll_com, yaw_com = setup_comutation.getAxisCalibrationParams(link)
    joint_y = setup_param.fetch_param(link, "GMB_OFF_JNT_Y").param_value
    joint_x = setup_param.fetch_param(link, "GMB_OFF_JNT_X").param_value
    joint_z = setup_param.fetch_param(link, "GMB_OFF_JNT_Z").param_value
    gyro_y = setup_param.fetch_param(link, "GMB_OFF_GYRO_Y").param_value
    gyro_x = setup_param.fetch_param(link, "GMB_OFF_GYRO_X").param_value
    gyro_z = setup_param.fetch_param(link, "GMB_OFF_GYRO_Z").param_value
    acc_y = setup_param.fetch_param(link, "GMB_OFF_ACC_Y").param_value
    acc_x = setup_param.fetch_param(link, "GMB_OFF_ACC_X").param_value
    acc_z = setup_param.fetch_param(link, "GMB_OFF_ACC_Z").param_value
    k_rate = setup_param.fetch_param(link, "GMB_K_RATE").param_value
    print "sw_ver, pitch_icept, pitch_slope, roll_icept, roll_slope, yaw_icept, yaw_slope, joint_y, joint_x, joint_z, gyro_y, gyro_x, gyro_z, acc_y, acc_x, acc_z, k_rate"
    print "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s" % (ver, pitch_com[1], pitch_com[2], roll_com[1], roll_com[2], yaw_com[1], yaw_com[2], joint_y, joint_x, joint_z, gyro_y, gyro_x, gyro_z, acc_y, acc_x, acc_z, k_rate)
    return ver