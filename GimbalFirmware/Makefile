
PROJ := PM_Sensorless_F2806x

PROJ_ROOT := .

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


###############################################################################
# common flags
#  XXX: in the eclipse build, these get passed when invoking the compiler and the linker,
#       though we should trim back as many of these as possible from the linker, since many
#       appear irrelevant for linking

COMMFLAGS += -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 -g -D=_DEBUG -D=LARGE_MODEL -D=FLOATING_MATH


###############################################################################
# C flags

CFLAGS  += -v28 -ml --float_support=fpu32
# XXX: turn this on once existing warnings are resolved
# CFLAGS  += --emit_warnings_as_errors
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

LDSCRIPTS := Source/F2806x_Headers_nonBIOS.cmd Source/F2806x_RAM_PM_Sensorless.CMD
STACK_SIZE := 0x380

LDFLAGS += $(COMMFLAGS)
LDFLAGS += -z -m$(PROJ).map --stack_size=$(STACK_SIZE) --warn_sections
LDFLAGS += -i$(TI_ROOT)/lib
LDFLAGS += --reread_libs --xml_link_info=$(PROJ)_linkinfo.xml --rom_model


###############################################################################
# Used libraries

LDLIBS += $(LIBS) $(LDSCRIPTS)


###############################################################################
# rules

all: $(PROJ).out $(PROJ).hex

$(PROJ).out: $(OBJS) $(LDSCRIPTS)
	$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $@

$(PROJ).hex: $(PROJ).out
	$(HEX) -i $< -o $@ -boot -gpio8 -b

%.o: %.asm
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -fe $@

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -fe $@

.PHONY: all clean

clean:
	rm -f $(PROJ).*
	rm -f $(OBJS)
	rm -f $(OBJS:.o=.pp)

-include $(OBJS:.o=.pp)
