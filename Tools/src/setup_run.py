#!/usr/bin/python

import os, sys, time, threading
from math import sin, cos, radians
from pymavlink.rotmat import Matrix3, Vector3
import setup_mavlink, setup_param, gui_graph

#import visual.graph
#import visual.crayola as color
#from visual.graph import gdisplay

testTargets = [
     Vector3(radians(-30), radians(-30), radians(-00)),
     Vector3(radians(-30), radians(+30), radians(-00)),
     Vector3(radians(+30), radians(+30), radians(-00)),
     Vector3(radians(+30), radians(-30), radians(-00)),
     Vector3(radians(-30), radians(-30), radians(-10)),
     Vector3(radians(-30), radians(+30), radians(-10)),
     Vector3(radians(+30), radians(+30), radians(-10)),
     Vector3(radians(+30), radians(-30), radians(-10)),
     Vector3(radians(-30), radians(-30), radians(+10)),
     Vector3(radians(-30), radians(+30), radians(+10)),
     Vector3(radians(+30), radians(+30), radians(+10)),
     Vector3(radians(+30), radians(-30), radians(+10))
]

class Log:
    def __init__(self):
        self.logdir = 'logs'
        self.mkdir_p(self.logdir)

        self.valuesLogfile = os.path.join(self.logdir, 'gyro_test_values_%d.csv' % time.time())
        self.valuesFile = open(self.valuesLogfile, 'w')
        self.valuesFile.write('time,rate_x,rate_y,rate_z,joint_x,joint_y,joint_z\n')

        self.eventsLogfile = os.path.join(self.logdir, 'gyro_test_events_%d.csv' % time.time())
        self.eventsFile = open(self.eventsLogfile, 'w')
        self.eventsFile.write('time,message\n')

    def mkdir_p(self, path):
        try:
            os.makedirs(path)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(path):
                pass
            else: raise

    def writeValues(self, measured_rate_corrected, measured_joint_corrected):
        log_str = "%s,%s,%s\n" % (time.time(), csvVector(measured_rate_corrected), csvVector(measured_joint_corrected))
        self.valuesFile.write(log_str)

    def writeEvent(self, message):
        log_str = "%s,%s\n" % (time.time(), message)
        self.eventsFile.write(log_str)

def csvVector(v):
    return '%f,%f,%f' % (v.x, v.y, v.z)

def niceExit(function):
    def wrapper(self, *args, **kwargs):
        try:
            return function(self, *args, **kwargs)
        except (KeyboardInterrupt, SystemExit):
            print('Exiting...')
            sys.exit(0)
    return wrapper

@niceExit
def runTest(link, test, stopTestsCallback=None, faultCallback=None, reportCallback=None):
    i = 0
    target = Vector3()
    log = None

    # For align and wobble tests
    if test in ['align', 'wobble']:
        joint_offsets = setup_param.get_offsets(link, 'JNT', timeout=4)
    
    # For wobble test
    if test == 'wobble':
        pointing_gain = 2
        start_time = time.time()
        gyro_offsets = setup_param.get_offsets(link, 'GYRO', timeout=4)
        error_integral = Vector3()
        log = Log()
        log.writeEvent('test started')

        #g1_r = visual.graph.gcurve(color=color.red)
        #g1_g = visual.graph.gcurve(color=color.green)
        #g1_b = visual.graph.gcurve(color=color.blue)
        #gdisplay()
        #g2_r = visual.graph.gcurve(color=color.red)
        #g2_g = visual.graph.gcurve(color=color.green)
        #g2_b = visual.graph.gcurve(color=color.blue)

    lastCycle = time.time()
    lastReport = time.time()
    commsLost = False
    while True:
        if stopTestsCallback:
            if stopTestsCallback():
                break

        # Receive all mavlink messages
        report = setup_mavlink.get_all(link, timeout=1)
        if report == None:
            continue
        elif report.get_type() == 'GIMBAL_REPORT':
            commsLost = False
            lastReport = time.time()
            delta = int(lastReport - lastCycle)
            if delta > 0:
                if log:
                    log.writeEvent('gimbal connected')
                if faultCallback:
                    faultCallback('gimbal connected')
        elif report.get_type() == 'STATUSTEXT':
            if log:
                log.writeEvent(report.text)
            if faultCallback:
                faultCallback(report.text)
            continue
        else:
            if (time.time() - lastReport) > 0.2 and not commsLost:
                commsLost = True
                if log:
                    log.writeEvent('gimbal reset')
                if faultCallback:
                    faultCallback('gimbal reset')
            continue

        if test == 'wobble':
            measured_rate = Vector3(report.delta_angle_x/report.delta_time , report.delta_angle_y/report.delta_time , report.delta_angle_z/report.delta_time)
            measured_rate_corrected = measured_rate - gyro_offsets/report.delta_time
            measured_joint = Vector3(report.joint_roll,report.joint_el,report.joint_az)
            measured_joint_corrected = measured_joint - joint_offsets

        Tvg = Matrix3()

        if test in ['align', 'wobble']:
            Tvg.from_euler312(report.joint_roll - joint_offsets.x, report.joint_el - joint_offsets.y, report.joint_az - joint_offsets.z)
        elif test == 'run':
            Tvg.from_euler312(report.joint_roll, report.joint_el, report.joint_az)
        current_angle = Vector3(*Tvg.to_euler312())

        if test == 'run':
            target.y = 0.4 * (sin(i * 12.5) - 1)
            target.x = 0.2 * cos(i * 2.5)
            target.z = 0.1 * cos(i * 0.5)
            rate = 5 * (target - current_angle)
        elif test == 'align':
            rate = 5 * (target - current_angle)
        elif test == 'wobble':
            error = target - current_angle
            error_integral = error+error_integral
            control_p = pointing_gain * (error)
            control_i = 0.00001 * (error_integral)
            rate = Tvg.transposed() * (control_p+control_i)

        if test in ['run', 'align']:
            setup_mavlink.send_gimbal_control(link, Tvg.transposed() * rate)
            i += 0.01
            #if reportCallback:
            #    reportCallback(report.joint_roll, report.joint_el, report.joint_az)
        elif test == 'wobble':
            setup_mavlink.send_gimbal_control(link, rate+gyro_offsets/report.delta_time)
            #print 'demanded '+csvVector(rate) +'\t measured '+ csvVector(measured_rate_corrected)+'\t joint '+ csvVector(measured_joint_corrected)

            if time.time() - start_time > 5:
                i = i + report.delta_time
                #g1_r.plot(pos=(i,measured_joint_corrected.x))
                #g1_g.plot(pos=(i,measured_joint_corrected.y))
                #g1_b.plot(pos=(i,measured_joint_corrected.z))
                #g2_r.plot(pos=(i,measured_rate_corrected.x))
                #g2_g.plot(pos=(i,measured_rate_corrected.y))
                #g2_b.plot(pos=(i,measured_rate_corrected.z))
                log.writeValues(measured_rate_corrected,measured_joint_corrected)

                if reportCallback:
                    reportCallback(measured_rate_corrected.x, measured_rate_corrected.y, measured_rate_corrected.z)

            if time.time() - start_time > 4:
                pointing_gain = 0.01

        current_angle = Vector3(*Tvg.to_euler312())
        lastCycle = time.time()

    if log:
        log.writeEvent('test started')

    return True

@niceExit    
def stop(link):
    rate = Vector3()
    while True:
        setup_mavlink.send_gimbal_control(link, rate)
