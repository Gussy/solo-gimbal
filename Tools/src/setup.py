#!/usr/bin/python

'''
Command-line utility to handle comms to gimbal 
'''
import time
import sys, argparse, time, json
import setup_factory_pub
from setup_mavlink import open_comm, wait_for_heartbeat

from firmware_helper import load_firmware, append_checksum
from firmware_loader import load_binary, start_bootloader
from firmware_loader import Results as loader_results
import setup_comutation, setup_mavlink
from setup_comutation import Results as calibration_results
import setup_validate, setup_param

# Optional imports
try:
    import setup_run
    import setup_home
    import setup_factory_private
except ImportError:
    setup_run = None
    setup_home = None
    setup_factory_private = None

def loaderProgressCallback(uploaded_kb, total_kb, percentage):
    sys.stdout.write("\rUpload %2.2fkB of %2.2fkB - %d%%     " % (uploaded_kb, total_kb, percentage))
    # The flush is required to refresh the screen on Ubuntu
    sys.stdout.flush()

def bootloaderVersionCallback(major, minor):
    print('Bootloader Ver %i.%i' % (major, minor))

def handle_file(args, link):
    fileExtension = str(args.file).split(".")[-1].lower()
    if fileExtension == 'param':
        setup_param.load_param_file(args.file, link)
    elif fileExtension == 'ax':
        # Prepare the binary to load from the compressed .ax file
        print('Application firmware_file: %s' % args.file)
        firmware = load_firmware(args.file)
        binary, checksum = append_checksum(firmware['binary'])
        print('Checksum: 0x%04X' % checksum)

        # Start the bootloader
        result = start_bootloader(link)
        if result == loader_results.NoResponse:
            print("No response from gimbal, exiting.")
            sys.exit(1)
        elif result == loader_results.InBoot:
            print('Target already in bootloader mode')
        elif result == loader_results.Restarting:
            print("Restarting target in bootloader mode")

        # Load the binary using the bootloader
        result = load_binary(binary, link, bootloaderVersionCallback=bootloaderVersionCallback, progressCallback=loaderProgressCallback)
        sys.stdout.write("\n")
        sys.stdout.flush();
        if result == loader_results.Success:
            print("Upload successful")
        elif result == loader_results.NoResponse:
            print("No response from gimbal, exiting.")
            sys.exit(1)
        elif result == loader_results.Timeout:
            print("Timeout")
            sys.exit(1)
        else:
            print("Unknown error while finishing bootloading")
            sys.exit(1)
    else:
        print("File type not supported")
        sys.exit(1)
    return

def calibrationProgressCallback(axis, progress, status):
    text = "\rCalibrating %s - progress %d%% - %s            " % (axis, progress, status)
    sys.stdout.write(text)
    # The flush is required to refresh the screen on Ubuntu
    sys.stdout.flush()

def eraseCalibration(link):
    print('Clearing old calibration values...')
    setup_comutation.resetCalibration(link)
    print('Calibration values cleared')

def runCalibration(link):
    result = setup_comutation.calibrate(link, calibrationProgressCallback)
    sys.stdout.write("\n")
    sys.stdout.flush()

    if result == calibration_results.ParamFetchFailed:
        print("Failed to get calibration parameters from the gimbal")
        return
    elif result == calibration_results.CommsFailed:
        print("Gimbal failed to communicate calibration progress")
        return

    # These results require a reset of the Gimbal
    if result == calibration_results.Success:
        print("Calibration successful!")
        return
    elif result == calibration_results.CalibrationExists:
        print("A calibration already exists, erase current calibration first (-e)")
        return
    elif result == calibration_results.PitchFailed:
        print("Pitch calibration failed")
        return
    elif result == calibration_results.RollFailed:
        print("Roll calibration failed")
        return
    elif result == calibration_results.YawFailed:
        print("Yaw calibration failed")
        return

    # Reset the gimbal
    print("Rebooting Gimbal")
    setup_mavlink.reset_gimbal(link)

    # Get the calibration data
    if result == calibration_results.Success:
        calibration = setup_comutation.getAxisCalibrationValues(link)
        if calibration:
            print("")
            for axis in ['pitch', 'roll', 'yaw']:
                print("%s: slope=%f intercept=%f" % (axis, calibration[axis]['slope'], calibration[axis]['intercept']))
        else:
            print("Error getting calibration values from the gimbal")
        return

def printValidation(link):
    valid = setup_validate.validate_version(link)
    if valid == setup_validate.Results.Pass:
        print("Version \t- PASS")
    elif valid == setup_validate.Results.Fail:
        print("Version \t- FAIL - please update to latest gimbal software")
    else:
        print("Version \t- ERROR")

    valid = setup_validate.validate_serial_number(link)
    if valid == setup_validate.Results.Pass:
        print("Serial Number \t- PASS")
    elif valid == setup_validate.Results.Fail:
        print("Serial Number \t- FAIL - Serial number was not set (--serialnumber SERIALNUMBER)")
    else:
        print("Serial Number \t- ERROR")

    valid = setup_validate.validate_date(link)
    if valid == setup_validate.Results.Pass:
        print("Assembly date \t- PASS")
    elif valid == setup_validate.Results.Fail:
        print("Assembly date \t- FAIL - assembly date was not set on the factory (--date)")
    else:
        print("Assembly date \t- ERROR")

    valid = setup_validate.validate_comutation(link)
    if valid == setup_validate.Results.Pass:
        print("Comutation \t- PASS")
    elif valid == setup_validate.Results.Fail:
        print("Comutation \t- FAIL - please redo the comutation calibration (-c)")
    else:
        print("Comutation \t- ERROR")

    valid = setup_validate.validate_joints(link)
    if valid == setup_validate.Results.Pass:
        print("Joints  \t- PASS")
    elif valid == setup_validate.Results.Fail:
        print("Joints  \t- FAIL - redo joint calibration (-j)")
    else:
        print("Joints  \t- ERROR")

    valid = setup_validate.validate_gyros(link)
    if valid == setup_validate.Results.Pass:
        print("Gyros   \t- PASS")
    elif valid == setup_validate.Results.Fail:
        print("Gyros   \t- FAIL - redo gyro calibration (-g)")
    else:
        print("Gyros   \t- ERROR")

    valid = setup_validate.validate_accelerometers(link)
    if valid == setup_validate.Results.Pass:
        print("Accelerometer\t- PASS")
    elif valid == setup_validate.Results.Fail:
        print("Accelerometer\t- FAIL - redo accelerometer calibration (-a)")
    else:
        print("Accelerometer\t- ERROR")

    valid = setup_validate.validate_gains(link)
    if valid == setup_validate.Results.Pass:
        print("Gains   \t- PASS")
    elif valid == setup_validate.Results.Fail:
        print("Gains   \t- FAIL - restore parameters to default values (-d)")
    else:
        print("Gains   \t- ERROR")

# Main method when called directly
def command_interface():
    parser = argparse.ArgumentParser()
    parser.add_argument("file",  nargs='?', help="parameter or firmware file to be loaded into the gimbal", default=None)
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading", default=None)
    parser.add_argument("-s", "--show", help="Show all useful gimbal parameters", action='store_true')
    parser.add_argument("-v", "--validate", help="Check gimbal parameters to see if they have valid values", action='store_true')
    parser.add_argument("-d", "--defaults", help="Reset gimbal parameters to default values", action='store_true')
    parser.add_argument("-r","--reboot", help="Reboot the gimbal", action='store_true')
    parser.add_argument("--eraseapp", help="Erase the application", action='store_true')
    parser.add_argument("-c", "--calibrate", help="Run the comutation setup", action='store_true')
    parser.add_argument("-f", "--forcecal", help="Force the comutation setup", action='store_true')
    parser.add_argument("-e", "--erase", help="Erase calibration values", action='store_true')
    parser.add_argument("--customgains", help="Enable the use of custom gains (0 to disable, 1 to enable)", type=int)

    # Optional commands (not included with sololink tools)
    if setup_run:
        parser.add_argument("--run", help="run a quick test of the gimbal", action='store_true')
        parser.add_argument("--align", help="move the gimbal to the home position", action='store_true')
        parser.add_argument("--stop", help="Hold the gimbal at the current position", action='store_true')
        parser.add_argument("--wobble", help="Wobble fixture test", action='store_true')
        parser.add_argument("--timeout", help="timeout for action", type=int)
        parser.add_argument("--testloop", help="run a loop of 'run', 'align' and 'wobble' tests", type=str)
        parser.add_argument("--lifetest", help="run the gimbal life test", action='store_true')
        parser.add_argument("--limp", help="disable position hold mode", action='store_true')
        parser.add_argument("--nolimp", help="enable position hold mode", action='store_true')
    if setup_home:
        parser.add_argument("-j", "--jointcalibration", help="Calibrate joint angles", action='store_true')
        parser.add_argument("-g", "--gyrocalibration", help="Calibrate gyros", action='store_true')
        parser.add_argument("-a", "--accelcalibration", help="Calibrate accelerometers", action='store_true')
        parser.add_argument("-x", "--staticcal", help="Calibrate all static home values", action='store_true')
    if setup_factory_private:
        parser.add_argument("--date", help="Setup assembly date", action='store_true')
        parser.add_argument("--serialnumber", help="Setup gimbal serial number", type=int)
        parser.add_argument("--factoryreset", help="Reset gimbal factory parameters to default", action='store_true')
        parser.add_argument("--fullreset", help="Clear all gimbal parameters and program memory", action='store_true')

    args = parser.parse_args()

    # Open the serial port
    port, link = open_comm(args.port)
    print("Connecting via port %s" % port)

    # Send a heartbeat first to wake up the interface, because mavlink
    link.heartbeat_send(0, 0, 0, 0, 0)

    # Attempt to get any gimbal message (usually the GOPRO_HEARTBEAT), then any bootloader messages
    # If neither attempts are successful, we're connected to a copter without a gimbal
    msg = setup_mavlink.get_gimbal_message(link)
    if msg is False:
        handshake = setup_mavlink.wait_handshake(link, retries=2)
        if handshake is None:
            print("No gimbal messages received, exiting.")
            sys.exit(1)
        elif not args.file:
            print("Gimbal in bootloader, only firmware loading is available")
            sys.exit(0)

    if args.file:
        handle_file(args, link)
        return

    if args.eraseapp:
        start_bootloader(link)
        print("Application erased")
        return

    if args.calibrate:
        runCalibration(link)
        return
    
    if args.forcecal:
        eraseCalibration(link)
        wait_for_heartbeat(link)
        # TODO: Check if this timeout is necessary
        time.sleep(5)
        runCalibration(link)
        return

    if args.show:
        params = setup_validate.show(link)
        if params:
            print(json.dumps(params, sort_keys=True, indent=4, separators=(',', ': ')))
        else:
            print("Error fetching parameters from gimbal")
        return
    
    if args.validate:
        print("Validating gimbal parameters...")
        printValidation(link)
        return
    
    if args.defaults:
        setup_validate.restore_defaults(link)
        print('Parameters restored to default values')
        return
        
    if args.reboot:
        if not setup_mavlink.reset_gimbal(link):
            print('Failed to reboot')
        return
    
    if args.erase:
        eraseCalibration(link)
        return

    if args.customgains == 1 or args.customgains == 0:
        setup_param.set_use_custom_gains(link, args.customgains)
        return

    if setup_run:
        if args.run:
            setup_run.runTest(link, 'run')
            return
        
        if args.align:
            setup_run.runTest(link, 'align')
            return

        if args.wobble:
            setup_run.runTest(link, 'wobble', timeout=args.timeout)
            return

        if args.testloop:
            if args.testloop in ['run', 'align', 'wobble']:
                setup_run.runTestLoop(link, args.testloop, timeout=args.timeout)
                return
            else:
                print("Unknown test: %s" % args.testloop)

        if args.lifetest:
            setup_run.runLifeTest(link)
            return

        if args.stop:
            setup_run.stop(link)
            return

        if args.limp:
            setup_param.pos_hold_disable(link)
            return

        if args.nolimp:
            setup_param.pos_hold_enable(link)
            return

    if setup_home:
        if args.jointcalibration or args.staticcal:
            print('Calibrating home position')
            offsets = setup_home.calibrate_joints(link)
            if offsets:
                print('Calibrated home position')
            else:
                print('Failed to calibrate home position')
            if not args.staticcal:
                return
        
        if args.gyrocalibration or args.staticcal:
            print('Calibrating gyro offsets')
            offsets = setup_home.calibrate_gyro(link)
            if offsets:
                print('Calibrated gyro offsets')
            else:
                print('Failed to calibrate gyro offsets')
            return
        
        if args.accelcalibration:
            setup_home.calibrate_accel(link)
            return

    if setup_factory_private:
        if args.date:
            timestamp = setup_factory_private.set_assembly_date(link)
            print("Assembly time set to %s" % time.ctime(timestamp))
            return

        if args.serialnumber is not None:
            serial = setup_factory_private.set_serial_number_3dr(link, args.serialnumber)
            print("Serial number set to %s" % serial)
            return

        if args.factoryreset:
            serial = setup_factory_private.reset(link)
            print("Facroty parameters cleared")
            return

        if args.fullreset:
            setup_factory_private.full_reset(link)
            print("Full reset command sent")
            return

    # Default command is to return the software version number
    ver = setup_factory_pub.read_software_version(link, timeout=4)
    if ver != None:
        major, minor, rev = ver[0], ver[1], ver[2]
        print("Software version: v%i.%i.%i" % (major, minor, rev))
        
        asm_time = setup_factory_pub.get_assembly_time(link)
        serial_number = setup_factory_pub.get_serial_number(link)
        if serial_number != None:
            if serial_number == '':
                print("Serial number: not set")
            else:
                print("Serial number: " + serial_number)
        else:
            print("Serial number: unknown")

        if asm_time != None:
            if asm_time > 0:
                print("Assembled time: " + time.ctime(asm_time))
            else:
                print("Assembly time: not set")
        else:
            print("Assembly time: unknown")
    else:
        print("Software version: unknown")

if __name__ == '__main__':
    command_interface()
    sys.exit(0)
