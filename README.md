# Solo Gimbal - AES
AES gimbal software for C2000 uC

## Development
Download and install the following softwares:
* [controlSUITE](http://www.ti.com/tool/controlsuite)
* [Code Composer Studio](http://www.ti.com/tool/ccstudio)

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

## Building bootloadable images

1. Run the build in Code Composer as normal.
2. Run ```GimbalFirmware\make_header.bat``` to convert ```.out``` into ```.bin``` hex file
3. (todo: package binary file up with python loading script)
