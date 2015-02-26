# Solo Gimbal - AES
AES gimbal software for C2000 uC

## Development
Download and install the following softwares:
* [controlSUITE](http://www.ti.com/tool/controlsuite)
* [Code Composer Studio](http://www.ti.com/tool/ccstudio)

### Requirements
* [Python](https://www.python.org/) must be installed and on the system PATH environment variable

Import the project into CCS via ```File>Import>C/C++>CCS Projects```, and browse for the root source folder

The active configuration should be FLASH. For that go to ```Project>Properties=>CCS General=>Manage Active Configurations``` select the FLASH option and click on ```Set Active```

Under targetConfigs folder right-click the file ```TMS320F28062.ccxml``` to set as the active configuration.

Under includes, open the file ```C:\ti\controlSUITE\development_kits/~/SupportFiles/F2806x_headers/PeripheralHeaderIncludes.h```

Replace the following snippet on line 94

```
#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int             int16;
typedef long            int32;
typedef unsigned int    Uint16;
typedef unsigned long   Uint32;
typedef float           float32;
typedef long double     float64;
#endif
```
with
```
#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef char            int8;
typedef int             int16;
typedef long            int32;
typedef unsigned char   Uint8;
typedef unsigned int    Uint16;
typedef unsigned long   Uint32;
typedef float           float32;
typedef long double     float64;
#endif
```

### For "GitHub for Windows" users
Gits executable is actually located in ```C:\Users\<user>\AppData\Local\GitHub\PortableGit_<guid>\bin\```

1. Right-Click on My Computer
2. Click Advanced System Settings
3. Click Environment Variables
4. Under System Variables find the path variable and click edit
5. Add the path to gits bin and cmd at the end of the string (like this: ```;C:\Users\<user>\AppData\Local\GitHub\PortableGit_<guid>\bin;C:\Users\<user>\AppData\Local\GitHub\PortableGit_<guid>\cmd```)
6. In the project properties, show advanced options, edit the ```GitDescribe``` configuration, change the ```Location``` to  ```C:\Users\<user>\AppData\Local\GitHub\PortableGit_<guid>\bin\sh.exe```

## Building

### GimbalFirmware

1. Build the ```GimbalFirmware``` project using Code Composer Studio.
2. (add steps for flashing GimbalFirmware only...?)

### AZBootloader

1. Build the ```GimbalFirmware``` project using Code Composer Studio.
2. Build the ```AZBootloader``` project using Code Composer Studio and flash to hardware using the XDS510 JTAG Emulator. During this process ```GimbalFirmware``` will be built to generate the application ```data.h``` payload to include with the ```AZBootloader```.

### Bootloader (CAN Bootstrapper)

1. (todo)

## Using MAVLink Bootloader

1. Build the ```GimbalFirmware``` project using Code Composer Studio.
2. Run the Python ```Tools\loadfw.py``` script (eg. ```python Tools\loadfw.py --port="COM7" GimbalFirmware\F2806x_RAM\PM_Sensorless_F2806x.hex```).

## Creating a release
1. Build the ```GimbalFirmware``` project using Code Composer Studio.
2. Tag the release using git and [SemVer](http://semver.org/) (eg ```git tag v1.0.5```)
3. Make a firmware release package using ```Tools\make_release.bat``` or ```Tools\make_release.sh``` (If the hardware revision has changed, update it in the ```Tools\make_release.*``` files)
4. Commit the firmware release and push it to GitHub
5. Use the GitHub release tool to publish a new release (using the latest file in ```Releases```)

# LED Patterns

## User LED (1 per board)

* Fast (3Hz) - No Comms
* Slow (0.8Hz) - Init
* Solid - Ready
* Error - Fast (3Hz), Pause after 3 cycles

## Beacon LED (Camera Carriage Board)

Currently cyces between R/G/B.
