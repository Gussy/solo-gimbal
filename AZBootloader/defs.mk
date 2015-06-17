
# we expect PROJ_ROOT and TI_ROOT to be defined by whoever included us

INCS = \
    -I=$(TI_ROOT)/include                                                               \
    -I=$(SHARED_ROOT)/                                                           		\
    -I=$(SHARED_ROOT)/headers                                                           \
    -I=$(SHARED_ROOT)/mavlink_library                                                   \
    -I=$(SHARED_ROOT)/mavlink_library/common                                            \
    -I=$(PROJ_ROOT)/../controlSUITE/device_support/f2806x/v100/F2806x_common/include    \
    -I=$(PROJ_ROOT)/../controlSUITE/device_support/f2806x/v100/F2806x_headers/include   \
    -I=$(PROJ_ROOT)/../controlSUITE/libs/utilities/flash_api/2806x/v100/include         \
    -I=$(PROJ_ROOT)/../controlSUITE/development_kits/~SupportFiles/F2806x_headers

OBJS = \
    $(SHARED_ROOT)/boot/ITRAPIsr.o 														\
    $(SHARED_ROOT)/boot/Init_Boot.o 													\
    $(SHARED_ROOT)/boot/Shared_Boot.o \
    $(SHARED_ROOT)/boot/Vectors_Boot.o 													\
    $(PROJ_ROOT)/main.o \
    $(SHARED_ROOT)/F2806x/source/F2806x_GlobalVariableDefs_patched.o

LIBS = \
    -l"libc.a" \
    -l2806x_BootROM_API_TABLE_Symbols_fpu32.lib
