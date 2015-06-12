################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
PID.obj: ../Source/control/PID.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/control/PID.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

average_power_filter.obj: ../Source/control/average_power_filter.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/control/average_power_filter.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

gyro_kinematics_correction.obj: ../Source/control/gyro_kinematics_correction.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/control/gyro_kinematics_correction.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

rate_loops.obj: ../Source/control/rate_loops.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/control/rate_loops.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

running_average_filter.obj: ../Source/control/running_average_filter.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/bin/cl2000" -v28 -ml --float_support=fpu32 -O4 --opt_for_speed=2 --include_path="/tmp/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/include" --include_path="../Headers/" --include_path="../Headers/MAVLink" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_headers/include" --include_path="../../controlSUITE/libs/math/IQmath/v15c/include" --include_path="../../controlSUITE/device_support/f2806x/v100/F2806x_common/include" --include_path="../../controlSUITE/development_kits/~SupportFiles/F2806x_headers" --include_path="../../controlSUITE/libs/app_libs/motor_control/math_blocks/v3.1" --include_path="../../controlSUITE/libs/utilities/flash_api/2806x/v100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" --quiet --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/control/running_average_filter.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


