#!/usr/bin/python

"""

"""
import sys
import setup_mavlink
from pymavlink.rotmat import Matrix3, Vector3
from math import sin, cos
import setup_param
from time import time

import visual.graph
import visual.crayola as color
from visual.graph import gdisplay
import math

class Log:
    def __init__(self):
        self.file = open('gyro_test_%d.csv'%time(),'w')
        self.file.write('time,rate_x,rate_y,rate_z,joint_x,joint_y,joint_z\n')

    def write(self, measured_rate_corrected, measured_joint_corrected):
        log_str = "%s,%s,%s\n"%(time(),csvVector(measured_rate_corrected),csvVector(measured_joint_corrected))
        self.file.write(log_str)
    

def csvVector(v):
    return '%f,%f,%f'%(v.x,v.y,v.z)

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
    start_time = time()
    i=0
    pointing_gain = 2
    gyro_offsets = setup_param.get_offsets(link, 'GYRO', timeout=1)
    joint_offsets = setup_param.get_offsets(link, 'JNT', timeout=1)
    target = Vector3()
    error_integral = Vector3()
    
    log = Log()
    
    g1_r = visual.graph.gcurve(color=color.red)
    g1_g = visual.graph.gcurve(color=color.green)
    g1_b = visual.graph.gcurve(color=color.blue)
    gdisplay()
    g2_r = visual.graph.gcurve(color=color.red)
    g2_g = visual.graph.gcurve(color=color.green)
    g2_b = visual.graph.gcurve(color=color.blue)
    
    while(True):

        
        report = setup_mavlink.get_gimbal_report(link, timeout=1)
        measured_rate = Vector3(report.delta_angle_x/report.delta_time , report.delta_angle_y/report.delta_time , report.delta_angle_z/report.delta_time)
        measured_rate_corrected = measured_rate - gyro_offsets/report.delta_time
        measured_joint = Vector3(report.joint_roll,report.joint_el,report.joint_az)
        measured_joint_corrected = measured_joint - joint_offsets


        Tvg = Matrix3()
        Tvg.from_euler312(report.joint_roll - joint_offsets.x, report.joint_el - joint_offsets.y, report.joint_az - joint_offsets.z)
        current_angle = Vector3(*Tvg.to_euler312())
                                   
        error = target - current_angle
        error_integral = error+error_integral
        control_p = pointing_gain * (error)
        control_i = 0.00001 * (error_integral)
        rate = Tvg.transposed() * (control_p+control_i)
        setup_mavlink.send_gimbal_control(link, rate+gyro_offsets/report.delta_time)
        
        print 'demanded '+csvVector(rate) +'\t measured '+ csvVector(measured_rate_corrected)+'\t joint '+ csvVector(measured_joint_corrected)
        
        if (time() - start_time>4):
            i = i + report.delta_time
            g1_r.plot(pos=(i,measured_joint_corrected.x))
            g1_g.plot(pos=(i,measured_joint_corrected.y))
            g1_b.plot(pos=(i,measured_joint_corrected.z))
            g2_r.plot(pos=(i,measured_rate_corrected.x))
            g2_g.plot(pos=(i,measured_rate_corrected.y))
            g2_b.plot(pos=(i,measured_rate_corrected.z))
            log.write(measured_rate_corrected,measured_joint_corrected)

        if (time() - start_time>3):
            pointing_gain = 0.01
            
def stop(link):
    rate = Vector3()
    while(True):        
        setup_mavlink.send_gimbal_control(link, rate)


