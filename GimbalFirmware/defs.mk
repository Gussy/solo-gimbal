
# we expect PROJ_ROOT and TI_ROOT to be defined by whoever included us

F2806X_COMMON := $(PROJ_ROOT)/../controlSUITE/device_support/f2806x/v100/F2806x_common/

INCS = \
    -I=$(TI_ROOT)/include                                                               \
    -I=$(PROJ_ROOT)/Headers/gopro                                                       \
    -I=$(PROJ_ROOT)/Headers                                                             \
    -I=$(SHARED_ROOT)/headers                                                           \
    -I=$(SHARED_ROOT)/mavlink_library                                                   \
    -I=$(PROJ_ROOT)/../controlSUITE/device_support/f2806x/v100/F2806x_headers/include   \
    -I=$(PROJ_ROOT)/../controlSUITE/libs/math/IQmath/v15c/include                       \
    -I=$(F2806X_COMMON)/include                                                         \
    -I=$(PROJ_ROOT)/../controlSUITE/development_kits/~SupportFiles/F2806x_headers       \
    -I=$(PROJ_ROOT)/../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1        \
    -I=$(PROJ_ROOT)/../controlSUITE/libs/utilities/flash_api/2806x/v100/include

OBJS = \
    $(PROJ_ROOT)/Source/can/can_message_processor.o 									\
    $(PROJ_ROOT)/Source/can/can_parameter_updates.o 									\
    $(PROJ_ROOT)/Source/can/cand.o 														\
    $(PROJ_ROOT)/Source/can/cb.o														\
    $(PROJ_ROOT)/Source/gopro/gopro_interface.o 										\
    $(PROJ_ROOT)/Source/gopro/gopro_i2c.o   \
    $(PROJ_ROOT)/Source/gopro/gopro_hero3p.o \
    $(PROJ_ROOT)/Source/gopro/gopro_hero4.o \
    $(PROJ_ROOT)/Source/control/average_power_filter.o 									\
    $(PROJ_ROOT)/Source/control/gyro_kinematics_correction.o 							\
    $(PROJ_ROOT)/Source/control/PID.o 													\
    $(PROJ_ROOT)/Source/control/rate_loops.o 											\
    $(PROJ_ROOT)/Source/control/running_average_filter.o 								\
    $(SHARED_ROOT)/src/flash/Example_Flash2806x_CsmKeys.o 								\
    $(SHARED_ROOT)/src/flash/flash.o 													\
    $(PROJ_ROOT)/Source/hardware/adc.o 													\
    $(SHARED_ROOT)/src/hardware/device_init.o 											\
    $(PROJ_ROOT)/Source/hardware/encoder.o 												\
    $(SHARED_ROOT)/src/hardware/gpio.o 													\
    $(PROJ_ROOT)/Source/hardware/gyro.o 												\
    $(PROJ_ROOT)/Source/hardware/HWSpecific.o 											\
    $(PROJ_ROOT)/Source/hardware/i2c.o 													\
    $(SHARED_ROOT)/src/hardware/led.o 													\
    $(PROJ_ROOT)/Source/hardware/spi.o 													\
    $(SHARED_ROOT)/src/hardware/uart.o 													\
    $(SHARED_ROOT)/src/hardware/pll.o 													\
    $(SHARED_ROOT)/src/hardware/watchdog.o												\
    $(SHARED_ROOT)/src/hardware/interrupts.o											\
    $(SHARED_ROOT)/src/flash/flash_init.o											    \
    $(PROJ_ROOT)/Source/helpers/fault_handling.o 										\
    $(PROJ_ROOT)/Source/helpers/ringbuf.o 												\
    $(PROJ_ROOT)/Source/mavlink_interface/mavlink_gimbal_interface.o 					\
    $(PROJ_ROOT)/Source/motor/commutation_calibration_state_machine.o 					\
    $(PROJ_ROOT)/Source/motor/motor_commutation.o 										\
    $(PROJ_ROOT)/Source/motor/motor_drive_state_machine.o 								\
    $(PROJ_ROOT)/Source/parameters/flash_params.o 										\
    $(PROJ_ROOT)/Source/parameters/load_axis_parms_state_machine.o 						\
    $(PROJ_ROOT)/Source/parameters/mavlink_parameter_interface.o 						\
    $(PROJ_ROOT)/Source/F2806x_GlobalVariableDefs.o 									\
    $(F2806X_COMMON)/source/F2806x_usDelay.o                                            \
    $(F2806X_COMMON)/source/F2806x_CodeStartBranch.o                                    \
    $(PROJ_ROOT)/PM_Sensorless.o

LIBS = \
    $(PROJ_ROOT)/libs/2806x_BootROM_API_TABLE_Symbols_fpu32.lib 						\
    $(PROJ_ROOT)/libs/IQmath.lib 														\
    $(PROJ_ROOT)/libs/IQmath_fpu32.lib 													\
    $(PROJ_ROOT)/libs/rts2800_fpu32_fast_supplement.lib
