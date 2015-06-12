################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
HWSpecific.obj: ../Source/hardware/HWSpecific.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/HWSpecific.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

adc.obj: ../Source/hardware/adc.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/adc.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

device_init.obj: ../Source/hardware/device_init.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/device_init.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

encoder.obj: ../Source/hardware/encoder.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/encoder.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

gpio.obj: ../Source/hardware/gpio.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/gpio.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

gyro.obj: ../Source/hardware/gyro.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/gyro.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

i2c.obj: ../Source/hardware/i2c.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/i2c.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

led.obj: ../Source/hardware/led.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/led.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

spi.obj: ../Source/hardware/spi.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/spi.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

uart.obj: ../Source/hardware/uart.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/hardware/uart.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


