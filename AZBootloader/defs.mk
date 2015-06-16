
# we expect PROJ_ROOT and TI_ROOT to be defined by whoever included us

INCS = \
    -I=$(TI_ROOT)/include                                                               \
    -I=$(PROJ_ROOT)/../controlSUITE/libs/utilities/flash_api/2806x/v100/include         \
    -I=$(PROJ_ROOT)/F2806x_headers/include                                              \
    -I=$(PROJ_ROOT)/../GimbalFirmware/Headers                                           \
    -I=$(PROJ_ROOT)/../GimbalFirmware/Headers/MAVLink                                   \
    -I=$(PROJ_ROOT)/../GimbalFirmware/Headers/MAVLink/common                            \
    -I=$(PROJ_ROOT)/../controlSUITE/device_support/f2806x/v100/F2806x_common/include    \
    -I=$(PROJ_ROOT)/../controlSUITE/development_kits/~SupportFiles/F2806x_headers

OBJS = \
    $(PROJ_ROOT)/ITRAPIsr.o \
    $(PROJ_ROOT)/Init_Boot.o \
    $(PROJ_ROOT)/Shared_Boot.o \
    $(PROJ_ROOT)/Vectors_Boot.o \
    $(PROJ_ROOT)/main.o \
    $(PROJ_ROOT)/F2806x_headers/source/F2806x_GlobalVariableDefs.o

LIBS = \
    -l"libc.a" \
    -l2806x_BootROM_API_TABLE_Symbols_fpu32.lib
