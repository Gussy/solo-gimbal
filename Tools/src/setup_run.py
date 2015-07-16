#!/usr/bin/python

import os, sys, time, threading
from math import sin, cos, radians
import math
from pymavlink.rotmat import Matrix3, Vector3
import setup_mavlink, setup_param, setup_factory, setup_validate
import fixtureWobble

#import visual.graph
#import visual.crayola as color
#from visual.graph import gdisplay

WOBBLE_TEST_ALIGNMENT_TIME = 5
RMS_WOBBLE_TEST_THRESHOLD = 0.005

stopTestLoop = False
faultCounter = 0
lastFaultTime = time.time()

class Log:
    def __init__(self, link, tag=''):
        self.logdir = 'logs'
        if not os.path.isdir(self.logdir):
            os.makedirs(self.logdir)

        self.logTimestamp = time.time()
        
        self.logSerialNumber = setup_factory.get_serial_number(link)
        if self.logSerialNumber == None:
            self.logSerialNumber = 'unknown'

        self.valuesLogfile = os.path.join(self.logdir, 'gyro_test_values_%s%s_%d.csv' % (self.logSerialNumber, tag, self.logTimestamp))
        self.valuesFile = open(self.valuesLogfile, 'w')
        self.valuesFile.write('time,rate_x,rate_y,rate_z,joint_x,joint_y,joint_z,min_x,min_y,min_z,max_x,max_y,max_z,rms_x,rms_y,rms_z\n')

        self.eventsLogfile = os.path.join(self.logdir, 'gyro_test_events_%s%s_%d.csv' % (self.logSerialNumber, tag, self.logTimestamp))
        self.eventsFile = open(self.eventsLogfile, 'w')
        self.eventsFile.write('time,message\n')
        
        self.limitsLogfile = os.path.join(self.logdir, 'gyro_test_limits_%s.csv' % (self.logSerialNumber))
        if os.path.exists(self.limitsLogfile):
            self.limitsFile = open(self.limitsLogfile, 'a')
        else:
            self.limitsFile = open(self.limitsLogfile, 'w')
            self.limitsFile.write('time,duration,min_x,min_y,min_z,max_x,max_y,max_z,rms_x,rms_y,rms_z,rms,rpm\n')

    def mkdir_p(self, path):
        try:
            os.makedirs(path)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(path):
                pass
            else: raise

    def writeValues(self, measured_rate_corrected, measured_joint_corrected, min, max, rms):
        log_str = "%s,%s,%s,%s,%s,%s\n" % (time.time(), csvVector(measured_rate_corrected), csvVector(measured_joint_corrected), csvVector(min), csvVector(max), csvVector(rms))
        self.valuesFile.write(log_str)

    def writeLimits(self, test_duration, min, max, rms, rpm):
        log_str = "%s,%s,%s,%s,%s,%f,%i\n" % (time.time(), test_duration, csvVector(min), csvVector(max), csvVector(rms), rms.length(), rpm)
        self.limitsFile.write(log_str)

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
def runLifeTest(link, stopTestsCallback=None, eventCallback=None, wobbleport=None):
    global stopTestLoop, faultCounter, lastFaultTime

    # Disable motors when the gimbal is in a fault state
    def eventCallbackShim(msg, fault=False):
        global stopTestLoop, faultCounter
        if fault:
            faultCounter += 1
            stopTestLoop = True

        # Call the original callback if there was one
        if eventCallback:
            eventCallback(msg, fault=fault)
        elif fault:
            print('Fault: %s' % msg)

    def stopCallbackShim():
        global stopTestLoop

        # Call the original callback if there was one
        if stopTestsCallback:
            # Handle a local stop (fault)
            if stopTestLoop:
                return True
            # Handle a remote stop (from gui)
            stopTestLoop = stopTestsCallback()
            return stopTestLoop
        else:
            return stopTestLoop

    faultCounter = 0
    maxFaults = 2
    timeBetweenFaults = 30

    speed = 116 # ~RPM
    timeout = None # Seconds
    #resetSecondsInerval = 1200 # 20 Minutes
    resetSecondsInerval = 60 # Seconds
    resetSecondsWait = 8 # Seconds

    wobble = fixtureWobble.init_fixture(wobbleport=wobbleport)

    i = 0
    while True:
        i += 1
        stopTestLoop = False

        if stopTestsCallback:
            if stopTestsCallback():
                break

        if eventCallback:
            eventCallback('Powering on gimbal')
        fixtureWobble.power_enable(wobble)
        time.sleep(resetSecondsWait)

        if eventCallback:
            eventCallback('Running test #%i' % i)
        runTest(
            link,
            'wobble',
            stopTestsCallback=stopCallbackShim,
            eventCallback=eventCallbackShim,
            timeout=resetSecondsInerval,
            rpm=speed,
            wobble=wobble
        )

        fixtureWobble.set_rpm(wobble, 0)

        now = time.time()
        if faultCounter >= maxFaults and (now - lastFaultTime) < timeBetweenFaults:
            if eventCallback:
                eventCallback('Stopping life test')
            break
        elif faultCounter >= maxFaults:
            if eventCallback:
                eventCallback('Resetting faultCounter')
            faultCounter = 1
            lastFaultTime = time.time()
        else:
            lastFaultTime = time.time()

        if eventCallback:
            eventCallback('Powering off gimbal')
        fixtureWobble.power_disable(wobble)
        time.sleep(1)

    # To be sure
    fixtureWobble.set_rpm(wobble, 0)
    fixtureWobble.power_enable(wobble)
    fixtureWobble.close(wobble)

    return True

@niceExit
def runTestLoop(link, test, stopTestsCallback=None, eventCallback=None, reportCallback=None, timeout=None, wobbleport=None):
    global stopTestLoop
    def eventCallbackShim(msg, fault=False):
        if fault:
            pass

        # Call the original callback if there was one
        if eventCallback:
            eventCallback(msg, fault=fault)

    def stopCallbackShim():
        global stopTestLoop
        # Call the original callback if there was one
        if stopTestsCallback:
            stopTestLoop = stopTestsCallback()
            return stopTestLoop

    wobble = fixtureWobble.init_fixture(wobbleport=wobbleport)

    currentSpeed = 0
    speeds = [116, 152, 161]
    for speed in speeds:
        currentSpeed = speed
        if stopTestsCallback is None:
            print("Running '%s' test at %i RPM" % (test, speed))
        runTest(link, test,
                stopTestsCallback=stopCallbackShim,
                eventCallback=eventCallbackShim,
                reportCallback=reportCallback,
                timeout=timeout,
                rpm=speed,
                wobble=wobble
                )
        if stopTestLoop:
            break

    # Soft stop
    for speed in range(currentSpeed):
        fixtureWobble.set_rpm(wobble, currentSpeed - speed)
    # To be sure
    fixtureWobble.set_rpm(wobble, 0)
    fixtureWobble.close(wobble)

    return True

@niceExit
def runTest(link, test, stopTestsCallback=None, eventCallback=None, reportCallback=None, timeout=None, rpm=None, wobble=None):
    global stopTestLoop
    i = 0
    target = Vector3()
    log = None

    start_time = time.time()
    if timeout is not None:
        timeout = timeout + WOBBLE_TEST_ALIGNMENT_TIME

    # For align and wobble tests
    if test in ['align', 'wobble']:
        joint_offsets = setup_param.get_offsets(link, 'JNT', timeout=4)
    
    # For wobble test
    if test == 'wobble':
        pointing_gain = 2
        gyro_offsets_raw = setup_param.get_offsets(link, 'GYRO', timeout=4)
        # Detect old gyro offsets
        if (gyro_offsets_raw.x < setup_validate.EXPECTED_GYRO / 100 and
            gyro_offsets_raw.y < setup_validate.EXPECTED_GYRO / 100 and
            gyro_offsets_raw.z < setup_validate.EXPECTED_GYRO / 100):
            msg = "Old gyro calibration detected, re-calibration required"
            if eventCallback:
                eventCallback(msg, fault=True)
            else:
                print(msg)
            stopTestLoop = True
            return
        gyro_offsets = gyro_offsets_raw / 100 # gyro offsets in rad/s
        error_integral = Vector3()
        if rpm:
            tag = '_%irpm' % rpm
        else:
            tag = None
        log = Log(link, tag=tag)
        log.writeEvent('test started')

        max = Vector3()
        min = Vector3()
        sqsum = Vector3()
        rms = Vector3()
        sample_count = 0

        #g1_r = visual.graph.gcurve(color=color.red)
        #g1_g = visual.graph.gcurve(color=color.green)
        #g1_b = visual.graph.gcurve(color=color.blue)
        #gdisplay()
        #g2_r = visual.graph.gcurve(color=color.red)
        #g2_g = visual.graph.gcurve(color=color.green)
        #g2_b = visual.graph.gcurve(color=color.blue)
        
        if wobble is None:
            _wobble = fixtureWobble.init_fixture()
        else:
            _wobble = wobble

    # Disable position hold mode
    setup_param.pos_hold_disable(link)

    lastCycle = time.time()
    lastReport = time.time()
    commsLost = False
    motorStarted = False
    while timeout is None or (time.time()-start_time) < timeout:
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
                if eventCallback:
                    eventCallback('gimbal connected')
        elif report.get_type() == 'STATUSTEXT':
            if log:
                log.writeEvent(report.text)
            if eventCallback:
                eventCallback(report.text, fault=True)
            continue
        else:
            if (time.time() - lastReport) > 2 and not commsLost:
                commsLost = True
                if log:
                    log.writeEvent('gimbal reset')
                if eventCallback:
                    eventCallback('gimbal reset', fault=True)
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

            # Wait for the gimbal t stabilise before starting the fixture motor
            if not motorStarted and time.time() - start_time > (WOBBLE_TEST_ALIGNMENT_TIME / 2):
                motorStarted = True
                fixtureWobble.set_rpm(_wobble, rpm)
                if rpm:
                    log.writeEvent('setting rpm to %i' % rpm)

            if time.time() - start_time > WOBBLE_TEST_ALIGNMENT_TIME:
                i = i + report.delta_time

                #g1_r.plot(pos=(i,measured_joint_corrected.x))
                #g1_g.plot(pos=(i,measured_joint_corrected.y))
                #g1_b.plot(pos=(i,measured_joint_corrected.z))
                #g2_r.plot(pos=(i,measured_rate_corrected.x))
                #g2_g.plot(pos=(i,measured_rate_corrected.y))
                #g2_b.plot(pos=(i,measured_rate_corrected.z))

                if(max.x < measured_rate_corrected.x):
                    max.x = measured_rate_corrected.x
                if(min.x > measured_rate_corrected.x):
                    min.x = measured_rate_corrected.x
                if(max.y < measured_rate_corrected.y):
                    max.y = measured_rate_corrected.y
                if(min.y > measured_rate_corrected.y):
                    min.y = measured_rate_corrected.y
                if(max.z < measured_rate_corrected.z):
                    max.z = measured_rate_corrected.z
                if(min.z > measured_rate_corrected.z):
                    min.z = measured_rate_corrected.z
                    
                sample_count = sample_count + 1
                sqsum.x = sqsum.x + measured_rate_corrected.x * measured_rate_corrected.x
                rms.x = (1.0 / sample_count) * math.sqrt(sqsum.x)
                sqsum.y = sqsum.y + measured_rate_corrected.y * measured_rate_corrected.y
                rms.y = (1.0 / sample_count) * math.sqrt(sqsum.y)
                sqsum.z = sqsum.z + measured_rate_corrected.z * measured_rate_corrected.z
                rms.z = (1.0 / sample_count) * math.sqrt(sqsum.z)
                    
                #print('max ' + str(max) + ' min ' + str(min) + ' rms ' + str(rms * 1000))

                log.writeValues(measured_rate_corrected, measured_joint_corrected, min, max, rms * 1000)

                if reportCallback:
                    reportCallback(measured_rate_corrected.x, measured_rate_corrected.y, measured_rate_corrected.z)

            if time.time() - start_time > 4:
                pointing_gain = 0.01

        current_angle = Vector3(*Tvg.to_euler312())
        lastCycle = time.time()

    # Re-enable position hold mode
    setup_param.pos_hold_enable(link)

    if log:
        if test == 'wobble':
            test_duration = int(time.time()-start_time - WOBBLE_TEST_ALIGNMENT_TIME)
            if rpm:
                log.writeLimits(test_duration, min, max, rms, rpm)
            else:
                log.writeLimits(test_duration, min, max, rms, 0)

            if rms.length() < RMS_WOBBLE_TEST_THRESHOLD:
                result = 'PASSED'
            else:
                result = 'FAILED'
            message = '%s wobble test - rms %.3f rad/s' % (result, rms.length())

            if eventCallback:
                eventCallback(message)
            else:
                print(message)

            if wobble is None:
                fixtureWobble.set_rpm(_wobble, 0)
                fixtureWobble.close(_wobble)
            
            
        log.writeEvent('test finished')
        return str(int(log.logTimestamp))

    return True

@niceExit    
def stop(link):
    rate = Vector3()
    while True:
        setup_mavlink.send_gimbal_control(link, rate)
