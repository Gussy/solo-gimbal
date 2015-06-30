#!/usr/bin/python

"""

"""
import sys
import setup_mavlink
from pymavlink.rotmat import Matrix3, Vector3
from math import sin, cos
import setup_param

def niceExit(function):
    def wrapper(self, *args, **kwargs):
        try:
            return function(self, *args, **kwargs)
        except (KeyboardInterrupt, SystemExit):
            print('Exiting...')
            sys.exit(0)
    return wrapper

@niceExit
def run(link):
    i = 0
    target = Vector3()
    while(True):
        report = setup_mavlink.get_gimbal_report(link, timeout=1)
        Tvg = Matrix3()
        Tvg.from_euler312( report.joint_roll, report.joint_el, report.joint_az)
        current_angle = Vector3(*Tvg.to_euler312())
        
        target.y = 0.4 * (sin(i * 12.5) - 1)
        target.x = 0.2 * cos(i * 2.5)
        target.z = 0.1 * cos(i * 0.5)
                           
        rate = 5 * (target - current_angle)
        
        setup_mavlink.send_gimbal_control(link, Tvg.transposed() * rate)
        i += 0.01

@niceExit
def align(link):
    i = 0
    offsets = setup_param.get_offsets(link, 'JNT', timeout=1)
    target = Vector3()
    while(True):
        report = setup_mavlink.get_gimbal_report(link, timeout=1)
        Tvg = Matrix3()
        Tvg.from_euler312( report.joint_roll - offsets.x, report.joint_el - offsets.y, report.joint_az - offsets.z)
        current_angle = Vector3(*Tvg.to_euler312())

        rate = 5 * (target-current_angle)
        
        setup_mavlink.send_gimbal_control(link, Tvg.transposed() * rate)
        i += 0.01

@niceExit
def wobble(link):
    i = 0
    gyro_offsets = setup_param.get_offsets(link, 'GYRO', timeout=1);
    offsets = setup_param.get_offsets(link, 'JNT', timeout=1)
    target = Vector3()
    
    #log = open('gyro_test_%d.txt'%time.time(),'w')
    
    while(True):
        report = setup_mavlink.get_gimbal_report(link, timeout=1)

        #log_str = "%1.2f \t%+02.3f\t%+02.3f\t%+02.3f\n"%(time.time(),report.delta_angle_x*1000.0,report.delta_angle_y*1000.0,report.delta_angle_z*1000.0)
        #print log_str
        #log.write(log_str)

        Tvg = Matrix3()
        Tvg.from_euler312(report.joint_roll - offsets.x, report.joint_el - offsets.y, report.joint_az - offsets.z)
        current_angle = Vector3(*Tvg.to_euler312())
                                   
        rate = 0.1 * (target - current_angle) + gyro_offsets
        
        setup_mavlink.send_gimbal_control(link, Tvg.transposed() * rate)
        i += 1

def stop(link):
    rate = Vector3()
    while(True):        
        setup_mavlink.send_gimbal_control(link, rate)


