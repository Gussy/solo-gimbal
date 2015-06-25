#!/usr/bin/python

'''
     Command-line utility to handle comms to gimbal 
'''
import sys, argparse, time, numpy, json

from firmware_loader import update
import setup_comutation, setup_home
from setup_mavlink import open_comm, wait_for_heartbeat
import setup_mavlink, setup_param
import setup_validate
import setup_run
from setup_param import set_offsets
import setup_factory

prettyPrint = False
outputJSON = False

def handle_file(args, link):
    fileExtension = str(args.file).split(".")[-1].lower()
    if fileExtension == 'param':
        setup_param.load_param_file(args.file, link)
    elif fileExtension == 'ax':
        update(args.file, link, outputHandler=printCallback)
    else:
        printErrorMessage("File type not supported")
        sys.exit(1)
    return

def printCallback(obj):
    global outputJSON
    if outputJSON:
        if 'message' in obj:
            obj['message'] = obj['message'].strip()
        printJSON(obj)
    elif 'message' in obj:
        sys.stdout.write(obj['message'])
        sys.stdout.flush()

def printJSON(obj):
    global prettyPrint
    if prettyPrint:
        print(json.dumps(obj, sort_keys=True, indent=4, separators=(',', ': ')))
    else:
        print(json.dumps(obj))

def printStatusMessage(msg, level='info'):
    global outputJSON
    if outputJSON:
        printJSON({
            'type': 'status',
            'level': level,
            'message': msg
        })
    else:
        print(msg)

def printErrorMessage(msg):
    printStatusMessage(msg, level='error')

def printSWVersion(version):
    global outputJSON
    major, minor, rev = version[0], version[1], version[2]
    if outputJSON:
        printJSON({
            'type': 'software_version',
            'major': major,
            'minor': minor,
            'rev': rev
        })
    else:
        print("Software version: v%i.%i.%i" % (major, minor, rev))

def printValidations(validations):
    global outputJSON
    if outputJSON:
        validations['type'] = 'validations'
        printJSON(validations)
    else:
        for component, result in validations.items():
            print(result['msg'])

def printAllParams(params):
    global outputJSON
    if outputJSON:
        params['type'] = 'all_parameters'
        printJSON(params)
    else:
        print(json.dumps(params, sort_keys=True, indent=4, separators=(',', ': ')))

def printOffsets(kind, offsets):
    global outputJSON
    if outputJSON:
        printJSON({
            'type': '%s_offsets' % kind,
            'offsets': {
                'x': offsets.x,
                'y': offsets.y,
                'z': offsets.z
            }
        })

# Main method when called directly
def command_interface():
    global prettyPrint, outputJSON

    parser = argparse.ArgumentParser()
    parser.add_argument("file",  nargs='?', help="parameter or firmware file to be loaded into the gimbal", default=None)
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading", default=None)
    parser.add_argument("--json", help="All output as JSON", action='store_true')
    parser.add_argument("--pretty", help="Pretty print JSON output", action='store_true')
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
 
    # Enable global JSON output
    if args.json:
        outputJSON = True

    # Enable global JSON pretty print
    if args.pretty:
        prettyPrint = True

    # Open the serial port
    port, link = open_comm(args.port, 230400)
    printStatusMessage("Connecting via port %s" % port)
    
    if wait_for_heartbeat(link) == None:
        printErrorMessage("Failed to comunicate to gimbal")
        sys.exit(1)
    else:
        printStatusMessage("Gimbal connected", level='wait')

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
        setup_comutation.calibrate(link, printCallback)
        return
    
    if args.forcecal:
        # Clear calibration values
        printStatusMessage('Clearing old calibration values', level='wait')
        setup_comutation.resetCalibration(link)
        printStatusMessage('Calibration values cleared', level='success')

        # Wait for gimbal to reset
        time.sleep(3)
        wait_for_heartbeat(link)

        # Run the new calibration
        setup_comutation.calibrate(link, printCallback)
        return
    
    if args.show:
        params = setup_validate.show(link)
        printAllParams(params)
        return
    
    if args.validate:
        printStatusMessage("Validating gimbal parameters", level='wait')
        result = setup_validate.validate(link)
        printValidations(result)
        return
    
    if args.defaults:
        setup_validate.restore_defaults(link)
        printStatusMessage('Parameters restored to default values', level='success')
        return
    
    if args.jointcalibration or args.staticcal:
        printStatusMessage('Calibrating home position', level='wait')
        offsets = setup_home.calibrate_joints(link)
        if offsets:
            printOffsets('joint', offsets)
        else:
            printErrorMessage('Failed to calibrate home position')
        if not args.staticcal:
            return
    
    if args.gyrocalibration or args.staticcal:
        printStatusMessage('Calibrating gyro offsets', level='wait')
        offsets = setup_home.calibrate_gyro(link)
        if offsets:
            printOffsets('gyro', offsets)
        else:
            printErrorMessage('Failed to calibrate gyro offsets')
        if not args.staticcal:
            return

    if args.staticcal:
        # Accelerometer calibration not implemented
        #set_offsets(link, 'ACC', numpy.zeros(3)) # Until we have accel cal zero the offsets on calibration
        return
    
    if args.accelcalibration:
        #setup_home.calibrate_accel(link)
        printErrorMessage('Accelerometer calibration not implemented')
        return
    
    if args.reboot:
        if not setup_mavlink.reset_gimbal(link):
            printErrorMessage('Failed to reboot')
        return
    
    if args.erase:
        printStatusMessage('Clearing old calibration values', level='wait')
        setup_comutation.resetCalibration(link)
        printStatusMessage('Calibration values cleared', level='success')
        return

    # Default command is to return the software version number
    ver = setup_factory.readSWver(link)
    if ver:
        asm_time = setup_factory.get_assembly_time(link)
        serial_number = setup_factory.get_serial_number(link)
        print("Software version: v%s" % ver)
        if(serial_number):    
            print "Serial number: "+ serial_number
        if(asm_time):
            print "Assembled on " +time.ctime(asm_time)
    else:
        print("Unable to read software version")
        sys.exit(1)

if __name__ == '__main__':
    command_interface()
    sys.exit(0)
