# Solo Gimbal - AES
AES gimbal software for C2000 uC

[![Build Status](https://magnum.travis-ci.com/3drobotics/solo-gimbal.svg?token=DrXtEFw3btp4K1aMV8zU&branch=master)](https://magnum.travis-ci.com/3drobotics/solo-gimbal)

## Development

The compiler tools can be downloaded for Linux, OS X, and Windows from [TI's website](http://software-dl.ti.com/codegen/non-esd/downloads/download.htm#C2000). 3DR keeps mirrors of a subset of compiler versions for convenient download:

* [linux (ti tools v6.4.2)](https://gimbal-ci.s3-website-us-east-1.amazonaws.com/compiler/ti-cgt-c2000_6.4.2.tar.gz)
* [os x (ti tools v6.4.2)](https://gimbal-ci.s3-website-us-east-1.amazonaws.com/compiler/ti_cgt_c2000_6.4.2_mac_installer.sh)
* windows...

TI's full-fledged eclipse-based developement environment can also be used:
* [Code Composer Studio](http://www.ti.com/tool/ccstudio) - also avaliable on [this mirror](https://gimbal-ci.s3-website-us-east-1.amazonaws.com/compiler/ti.tar.gz)
* Import the project into CCS via ```File>Import>C/C++>CCS Projects```, and browse for the root source folder

### Requirements

* [Python](https://www.python.org/) must be installed and on the system PATH environment variable

For benchtop development, you'll likely want a custom cable that integrates cables from a power supply (16.8V, current limit can be around 0.5A for just GoPro comms, or 3.0A to drive all the motors), and an FTDI cable onto the custom connector that plugs into the AZ board. todo: provide wiring diagram...

## Building

The following folders are each individual projects that generate a binary:

* `GimbalFirmware`: the main application firmware image that runs on all 3 boards
* `AZBootloader`: the bootloader that runs on the AZ board, and speaks mavlink to the autopilot - this must be loaded onto the AZ board
* `Bootloader`: the CAN bootloader that is updated via the AZBootloader - this must be loaded onto the EL and ROLL boards

navigate to any of these folders, and run `make` to build, or open them in Code Composer Studio and hit the Build button. You can also run `make` from the top level folder to build all projects.

## Flashing

The `gimbal_firmware_*.ax` image, generated by the `GimbalFirmware` project, can be loaded over uart/usb serial. Run `python Tools/src/setup.py -h` and follow the instructions there.

The bootloaders must be installed via JTAG, usually using the XDS510 JTAG Emulator from within Code Composer Studio.

## LED Patterns

### User LED (1 per board)
* BLINK_NO_COMM - fast ,3Hz, duty cycle of 50%
* BLINK_ERROR - fast, 3Hz, duty cycle 50%, pause after 3 cycles - system in an error state, usually an over-current
* BLINK_INIT - slow, .8Hz, dudy cycle of 20%  - system being initialized
* BLINK_READY - slow, .5Hz , dudy cycle of 90% - system initialized but idle, since there are no rate commands
* BLINK_RUNNING - on all the time  - system working

### Beacon LED (Camera Carriage Board)

* Blinking white on Bootloader
* Glows green on power up for a couple of seconds

## Misc

As the gimbal speaks mavlink, you can connect to it via [mavproxy](https://tridge.github.io/MAVProxy) for debugging and general development:

    mavproxy.py --master=/dev/ttyUSB0,230400 --nowait --target-system=0
