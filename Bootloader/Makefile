
PROJ := Bootloader

PROJ_ROOT := .
SHARED_ROOT := ../shared
PROJ_PYTHON_ROOT := $(PROJ_ROOT)/../Tools/src

ifeq ($(TI_ROOT),)
# XXX: would really prefer to not require this but at least for now,
# we need to explicitly include paths in the compiler's directory path
# to get access to things like stdint.h and libc.a
$(error TI_ROOT not set, please export path to TI tools (`export TI_ROOT=/path/to/tools`).)
endif

include defs.mk

# Disable implicit rules - we don't rely on them, and this is faster.
.SUFFIXES:

JTAG_TOOL := XDS500
#JTAG_TOOL := XDS100v3

###############################################################################
# executables

CC      := $(TI_ROOT)/bin/cl2000
LD      := $(TI_ROOT)/bin/cl2000
HEX     := $(TI_ROOT)/bin/hex2000
TILOAD  := $(TI_ROOT)../../../ccs_base/scripting/examples/loadti/loadti.sh

SIZE_INFO	:= python $(PROJ_PYTHON_ROOT)/firmware_size_info.py

###############################################################################
# common flags
#  XXX: in the eclipse build, these get passed when invoking the compiler and the linker,
#       though we should trim back as many of these as possible from the linker, since many
#       appear irrelevant for linking

COMMFLAGS += -v28 -ml -mt --float_support=fpu32 -O3 --opt_for_speed=2 -g -D=BOOTLOADER -D=_DEBUG -D=LARGE_MODEL -D=FLOATING_MATH
COMMFLAGS += -DMAVLINK_COMM_NUM_BUFFERS=1


###############################################################################
# C flags

CFLAGS  += -v28 -ml --float_support=fpu32
CFLAGS  += --emit_warnings_as_errors
CFLAGS  += --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency=$(@:.o=.pp)
CFLAGS  += --c99 --issue_remarks


###############################################################################
# C & C++ preprocessor common flags

CPPFLAGS  += $(COMMFLAGS)
CPPFLAGS  += $(INCS)


###############################################################################
# C++ flags (not supported by cl2000)

CXXFLAGS  += 


###############################################################################
# Linker flags

LDSCRIPTS := 28067_lnk.cmd
STACK_SIZE := 0x200

LDFLAGS += $(COMMFLAGS)
LDFLAGS += -z --stack_size=$(STACK_SIZE) --warn_sections
LDFLAGS += -i$(TI_ROOT)/lib -i$(PROJ_ROOT)/../controlSUITE/libs/utilities/flash_api/2806x/v100/lib
LDFLAGS += --reread_libs --xml_link_info=$(PROJ)_linkinfo.xml --rom_model
LDFLAGS += --warn_sections


###############################################################################
# other deps

# build result from GimbalFirmware, we just expect this to already be built for now
MAIN_FW := $(PROJ_ROOT)/../GimbalFirmware/PM_Sensorless_F2806x.out

# JTAG Programmer configuration file
JTAG_CONFIG	:= $(PROJ_ROOT)/targetConfigs/TMS320F28067_$(JTAG_TOOL).ccxml

###############################################################################
# Used libraries

LDLIBS += $(LIBS) $(LDSCRIPTS)


###############################################################################
# rules

all: $(PROJ).hex

flash: all
	$(TILOAD) -a -c=$(JTAG_CONFIG) $(PROJ).hex

$(PROJ).out: $(OBJS) $(LDSCRIPTS)
	$(LD) $(LDFLAGS) -m$(PROJ).map $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $@

$(PROJ).hex: $(PROJ).out
	$(HEX) -i $< -o $@ -order MS -romwidth 16
	$(SIZE_INFO) $(PROJ).map --project=$(PROJ)

%.o: %.asm
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -fe $@

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -fe $@

.PHONY: all clean flash

clean:
	rm -f $(PROJ).* $(PROJ)_linkinfo.xml
	rm -f $(OBJS)
	rm -f $(OBJS:.o=.pp)

-include $(OBJS:.o=.pp)
