#!/usr/bin/python

"""

"""
import setup_mavlink
from pymavlink.rotmat import Matrix3, Vector3
from math import sin, cos

def run(link):
    i=0
    target = Vector3()
    while(True):
        report = setup_mavlink.get_gimbal_report(link)
        Tvg = Matrix3()
        Tvg.from_euler312( report.joint_roll, report.joint_el, report.joint_az)
        current_angle = Vector3(*Tvg.to_euler312())
        
        target.y = 0.4*(sin(i*12.5)-1)
        target.x = 0.2*cos(i*2.5) 
        target.z = 0.1*cos(i*0.5) 
                           
        rate = 5*(target-current_angle)
        
        setup_mavlink.send_gimbal_control(link, Tvg.transposed()*rate)
        i+=0.01