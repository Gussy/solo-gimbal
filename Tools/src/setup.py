#!/usr/bin/python

'''
     Command-line utility to handle comms to gimbal 
'''
import sys, argparse, time, numpy, json

from firmware_loader import prepare_binary, load_binary, start_bootloader
from firmware_loader import Results as loader_results
import setup_comutation, setup_home
from setup_comutation import Results as calibration_results
from setup_mavlink import open_comm, wait_for_heartbeat
import setup_mavlink, setup_param
import setup_validate
import setup_run
from setup_param import set_offsets
import setup_factory

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
        binary, checksum = prepare_binary(args.file)
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
    if valid is True:
        print("Version \t- PASS")
    elif valid is False:
        print("Version \t- FAIL - please update to latest gimbal software")
    else:
        print("Version \t- ERROR")

    valid = setup_validate.validate_comutation(link)
    if valid is True:
        print("Comutation \t- PASS")
    elif valid is False:
        print("Comutation \t- FAIL - please redo the comutation calibration (-c)")
    else:
        print("Comutation \t- ERROR")

    valid = setup_validate.validate_joints(link)
    if valid is True:
        print("Joints  \t- PASS")
    elif valid is False:
        print("Joints  \t- FAIL - redo joint calibration (-j)")
    else:
        print("Joints  \t- ERROR")

    valid = setup_validate.validate_gyros(link)
    if valid is True:
        print("Gyros   \t- PASS")
    elif valid is False:
        print("Gyros   \t- FAIL - redo gyro calibration (-g)")
    else:
        print("Gyros   \t- ERROR")

    valid = setup_validate.validate_accelerometers(link)
    if valid is True:
        print("Accelerometer\t- PASS")
    elif valid is False:
        print("Accelerometer\t- FAIL - redo accelerometer calibration (-a)")
    else:
        print("Accelerometer\t- ERROR")

    valid = setup_validate.validate_gains(link)
    if valid is True:
        print("Gains   \t- PASS")
    elif valid is False:
        print("Gains   \t- FAIL - restore parameters to default values (-d)")
    else:
        print("Gains   \t- ERROR")

# Main method when called directly
def command_interface():
    global prettyPrint, outputJSON

    parser = argparse.ArgumentParser()
    parser.add_argument("file",  nargs='?', help="parameter or firmware file to be loaded into the gimbal", default=None)
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading", default=None)
    parser.add_argument("-s", "--show", help="Show all useful gimbal parameters", action='store_true')
    parser.add_argument("-v", "--validate", help="Check gimbal parameters to see if they have valid values", action='store_true')
    parser.add_argument("-d", "--defaults", help="Reset gimbal parameters to default values", action='store_true')
    parser.add_argument("-r","--reboot", help="Reboot the gimbal", action='store_true')
    parser.add_argument("--run", help="run a quick test of the gimbal", action='store_true')
    parser.add_argument("--align", help="move the gimbal to the home position", action='store_true')
    parser.add_argument("--stop", help="Hold the gimbal at the current position", action='store_true')
    parser.add_argument("-c", "--calibrate", help="Run the comutation setup", action='store_true')
    parser.add_argument("-f", "--forcecal", help="Force the comutation setup", action='store_true')
    parser.add_argument("-j", "--jointcalibration", help="Calibrate joint angles", action='store_true')
    parser.add_argument("-g", "--gyrocalibration", help="Calibrate gyros", action='store_true')
    parser.add_argument("-a", "--accelcalibration", help="Calibrate accelerometers", action='store_true')
    parser.add_argument("-x", "--staticcal", help="Calibrate all static home values", action='store_true')
    parser.add_argument("-e", "--erase", help="Erase calibration values", action='store_true')
    parser.add_argument("--date", help="Setup assembly date", action='store_true')
    parser.add_argument("--serialnumber", help="Setup gimbal serial number", type=int)    
    args = parser.parse_args()

    # Open the serial port
    port, link = open_comm(args.port)
    print("Connecting via port %s" % port)

    if wait_for_heartbeat(link) == None:
        print("Failed to comunicate to gimbal")
        sys.exit(1)
    else:
        print("Gimbal connected")

    if args.file:
        handle_file(args, link)
        return

    if args.date:
        setup_factory.set_assembly_date(link)
        return

    if args.serialnumber is not None:
        setup_factory.set_serial_number_3dr(link, args.serialnumber)
        return
    
    if args.run:
        setup_run.run(link)
        return
    
    if args.stop:
        setup_run.stop(link)
        return
    
    if args.align:
        setup_run.align(link)
        return
    
    if args.calibrate:
        runCalibration(link)
        return
    
    if args.forcecal:
        eraseCalibration(link)
        wait_for_heartbeat(link)
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
    
    if args.jointcalibration or args.staticcal:
        print('Calibrating home position')
        offsets = setup_home.calibrate_joints(link)
        if offsets:
            print('joint', offsets)
        else:
            print('Failed to calibrate home position')
        if not args.staticcal:
            return
    
    if args.gyrocalibration or args.staticcal:
        print('Calibrating gyro offsets')
        offsets = setup_home.calibrate_gyro(link)
        if offsets:
            print('gyro', offsets)
        else:
            print('Failed to calibrate gyro offsets')
        if not args.staticcal:
            return

    if args.staticcal:
        # Accelerometer calibration not implemented
        #set_offsets(link, 'ACC', numpy.zeros(3)) # Until we have accel cal zero the offsets on calibration
        return
    
    if args.accelcalibration:
        #setup_home.calibrate_accel(link)
        print('Accelerometer calibration not implemented')
        return
    
    if args.reboot:
        if not setup_mavlink.reset_gimbal(link):
            print('Failed to reboot')
        return
    
    if args.erase:
        eraseCalibration(link)
        return

    # Default command is to return the software version number
    ver = setup_factory.readSWver(link)
    asm_time = setup_factory.get_assembly_time(link)
    serial_number = setup_factory.get_serial_number(link)
    if ver:
        major, minor, rev = ver[0], ver[1], ver[2]
        print("Software version: v%i.%i.%i" % (major, minor, rev))
    else:
        print("Unable to read software version")
        sys.exit(1)
    if serial_number:    
        print("Serial number: " + serial_number)
    if asm_time:
        print("Assembled on " + time.ctime(asm_time))

if __name__ == '__main__':
    command_interface()
    sys.exit(0)
