################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2

ORDERED_OBJS += \
-l"rts2800_fpu32.lib" \
"./PM_Sensorless.obj" \
"./Example_Flash2806x_CsmKeys.obj" \
"./flash_params.obj" \
"./load_axis_parms_state_machine.obj" \
"./mavlink_parameter_interface.obj" \
"./commutation_calibration_state_machine.obj" \
"./motor_commutation.obj" \
"./motor_drive_state_machine.obj" \
"./mavlink_gimbal_interface.obj" \
"./fault_handling.obj" \
"./ringbuf.obj" \
"./HWSpecific.obj" \
"./adc.obj" \
"./device_init.obj" \
"./encoder.obj" \
"./gpio.obj" \
"./gyro.obj" \
"./i2c.obj" \
"./led.obj" \
"./spi.obj" \
"./uart.obj" \
"./gopro_interface.obj" \
"./PID.obj" \
"./average_power_filter.obj" \
"./gyro_kinematics_correction.obj" \
"./rate_loops.obj" \
"./running_average_filter.obj" \
"./can_message_processor.obj" \
"./can_parameter_updates.obj" \
"./cand.obj" \
"./cb.obj" \
"./F2806x_CodeStartBranch.obj" \
"./F2806x_GlobalVariableDefs.obj" \
"./F2806x_usDelay.obj" \
"../libs/2806x_BootROM_API_TABLE_Symbols_fpu32.lib" \
"../libs/IQmath.lib" \
"../libs/IQmath_fpu32.lib" \
"../libs/rts2800_fpu32_fast_supplement.lib" \
"../Source/F2806x_Headers_nonBIOS.cmd" \
"../Source/F2806x_RAM_PM_Sensorless.CMD" \
$(GEN_CMDS__FLAG) \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include libs/subdir_vars.mk
-include Source/parameters/subdir_vars.mk
-include Source/motor/subdir_vars.mk
-include Source/mavlink_interface/subdir_vars.mk
-include Source/helpers/subdir_vars.mk
-include Source/hardware/subdir_vars.mk
-include Source/gopro/subdir_vars.mk
-include Source/control/subdir_vars.mk
-include Source/can/subdir_vars.mk
-include Source/subdir_vars.mk
-include subdir_rules.mk
-include libs/subdir_rules.mk
-include Source/parameters/subdir_rules.mk
-include Source/motor/subdir_rules.mk
-include Source/mavlink_interface/subdir_rules.mk
-include Source/helpers/subdir_rules.mk
-include Source/hardware/subdir_rules.mk
-include Source/gopro/subdir_rules.mk
-include Source/control/subdir_rules.mk
-include Source/can/subdir_rules.mk
-include Source/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
PM_Sensorless_F2806x.out \

EXE_OUTPUTS__QUOTED += \
"PM_Sensorless_F2806x.out" \


# All Target
all:
	@$(MAKE) --no-print-directory pre-build
	@$(MAKE) --no-print-directory main-build

# Main-build Target
main-build: PM_Sensorless_F2806x.out

# Tool invocations
PM_Sensorless_F2806x.out: $(OBJS) $(CMD_SRCS) $(CMD_UPPER_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 -z -m"PM_Sensorless_F2806x.map" --stack_size=0x380 --warn_sections -i"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/lib" -i"/home/arthur/solo/solo-gimbal/GimbalFirmware" -i"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --reread_libs --xml_link_info="PM_Sensorless_F2806x_linkInfo.xml" --rom_model -o "PM_Sensorless_F2806x.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "PM_Sensorless.pp" "Source/parameters/flash_params.pp" "Source/parameters/load_axis_parms_state_machine.pp" "Source/parameters/mavlink_parameter_interface.pp" "Source/motor/commutation_calibration_state_machine.pp" "Source/motor/motor_commutation.pp" "Source/motor/motor_drive_state_machine.pp" "Source/mavlink_interface/mavlink_gimbal_interface.pp" "Source/helpers/fault_handling.pp" "Source/helpers/ringbuf.pp" "Source/hardware/HWSpecific.pp" "Source/hardware/adc.pp" "Source/hardware/device_init.pp" "Source/hardware/encoder.pp" "Source/hardware/gpio.pp" "Source/hardware/gyro.pp" "Source/hardware/i2c.pp" "Source/hardware/led.pp" "Source/hardware/spi.pp" "Source/hardware/uart.pp" "Source/gopro/gopro_interface.pp" "Source/control/PID.pp" "Source/control/average_power_filter.pp" "Source/control/gyro_kinematics_correction.pp" "Source/control/rate_loops.pp" "Source/control/running_average_filter.pp" "Source/can/can_message_processor.pp" "Source/can/can_parameter_updates.pp" "Source/can/cand.pp" 
	-$(RM) "Source/can/cb.pp" "Source/F2806x_GlobalVariableDefs.pp" 
	-$(RM) "PM_Sensorless.obj" "Example_Flash2806x_CsmKeys.obj" "flash_params.obj" "load_axis_parms_state_machine.obj" "mavlink_parameter_interface.obj" "commutation_calibration_state_machine.obj" "motor_commutation.obj" "motor_drive_state_machine.obj" "mavlink_gimbal_interface.obj" "fault_handling.obj" "ringbuf.obj" "HWSpecific.obj" "adc.obj" "device_init.obj" "encoder.obj" "gpio.obj" "gyro.obj" "i2c.obj" "led.obj" "spi.obj" "uart.obj" "gopro_interface.obj" "PID.obj" "average_power_filter.obj" "gyro_kinematics_correction.obj" "rate_loops.obj" "running_average_filter.obj" "can_message_processor.obj" "can_parameter_updates.obj" "cand.obj" "cb.obj" "F2806x_CodeStartBranch.obj" "F2806x_GlobalVariableDefs.obj" "F2806x_usDelay.obj" 
	-$(RM) "Source/parameters/Example_Flash2806x_CsmKeys.pp" "Source/F2806x_CodeStartBranch.pp" "Source/F2806x_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

pre-build:
	-cmd /C "del PM_Sensorless_F2806x.out"
	-cmd /C "del PM_Sensorless_F2806x.hex"
	-python "../../Tools/src/firmware_version_header.py" "../Headers/version.h"
	-@echo ' '

post-build:
	-@echo 'Build hex binary'
	-"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/hex2000" -i "PM_Sensorless_F2806x.out" -o "PM_Sensorless_F2806x.hex" -boot -gpio8 -b
	-python ../../Tools/src/firmware_release.py --board_revision=0 --image=./PM_Sensorless_F2806x.hex --outdir=../../
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: main-build pre-build post-build

-include ../makefile.targets

