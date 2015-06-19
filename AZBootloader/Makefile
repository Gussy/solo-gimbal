
PROJ := AZBootloader

PROJ_ROOT := .
SHARED_ROOT := ../shared

ifeq ($(TI_ROOT),)
# XXX: would really prefer to not require this but at least for now,
# we need to explicitly include paths in the compiler's directory path
# to get access to things like stdint.h and libc.a
$(error TI_ROOT not set, please export path to TI tools (`export TI_ROOT=/path/to/tools`).)
endif

include defs.mk

# Disable implicit rules - we don't rely on them, and this is faster.
.SUFFIXES:


###############################################################################
# executables

CC      := $(TI_ROOT)/bin/cl2000
LD      := $(TI_ROOT)/bin/cl2000
HEX     := $(TI_ROOT)/bin/hex2000
TILOAD  := $(TI_ROOT)../../../ccs_base/scripting/examples/loadti/loadti.sh

###############################################################################
# common flags
#  XXX: in the eclipse build, these get passed when invoking the compiler and the linker,
#       though we should trim back as many of these as possible from the linker, since many
#       appear irrelevant for linking

COMMFLAGS += -v28 -ml -mt --float_support=fpu32 -O4 --opt_for_speed=2 -g -D=_DEBUG -D=LARGE_MODEL -D=FLOATING_MATH


###############################################################################
# C flags

CFLAGS  += -v28 -ml --float_support=fpu32
CFLAGS  += --emit_warnings_as_errors
CFLAGS  += --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency=$(@:.o=.pp)


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
LDFLAGS += -z -m$(PROJ).map --stack_size=$(STACK_SIZE) --warn_sections
LDFLAGS += -i$(TI_ROOT)/lib -i$(PROJ_ROOT)/../controlSUITE/libs/utilities/flash_api/2806x/v100/lib
LDFLAGS += --reread_libs --xml_link_info=$(PROJ)_linkinfo.xml --rom_model


###############################################################################
# other deps

# build result from GimbalFirmware, we just expect this to already be built for now
MAIN_FW := $(PROJ_ROOT)/../GimbalFirmware/PM_Sensorless_F2806x.out
DATA_HDR := data.h

# JTAG Programmer configuration file
JTAG_CONFIG	:= $(PROJ_ROOT)/targetConfigs/TMS320F28067.ccxml

###############################################################################
# Used libraries

LDLIBS += $(LIBS) $(LDSCRIPTS)


###############################################################################
# rules

all: $(PROJ).hex

flash: all
	$(TILOAD) -a -c=$(JTAG_CONFIG) $(PROJ).hex

# generate data header from main fw image
$(DATA_HDR): $(MAIN_FW)
	$(HEX) $(MAIN_FW) -o data.bin -boot -gpio8 -b
	python $(PROJ_ROOT)/../Tools/src/firmware_to_header.py data.bin $@
	rm data.bin

# order-only prerequisite, to ensure $(DATA_HDR) is built before any source files that might depend on it
# http://www.gnu.org/software/make/manual/make.html#Prerequisite-Types
$(OBJS): | $(DATA_HDR)

$(PROJ).out: $(OBJS) $(LDSCRIPTS)
	$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $@

$(PROJ).hex: $(PROJ).out
	$(HEX) -i $< -o $@ -order MS -romwidth 16

%.o: %.asm
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -fe $@

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -fe $@

.PHONY: all clean flash

clean:
	rm -f $(PROJ).* $(PROJ)_linkinfo.xml
	rm -f $(OBJS)
	rm -f $(OBJS:.o=.pp)
	rm -f $(DATA_HDR)

-include $(OBJS:.o=.pp)
