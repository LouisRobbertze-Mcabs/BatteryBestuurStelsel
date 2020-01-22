################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
DSP2803x_CodeStartBranch.obj: C:/Users/Bartho\ Horn/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_6.2.5/bin/cl2000" -v28 -ml -mt --cla_support=cla0 -g --include_path="C:/ti/ccsv6/tools/compiler/c2000_6.2.5/include" --include_path="/packages/ti/xdais" --include_path="C:/Users/Bartho Horn/Documents/Github/BatteryBestuurStelsel/DSP2803x_headers/include" --include_path="C:/Users/Bartho Horn/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/include" --include_path="C:/Users/Bartho Horn/libs/math/IQmath/v160/include" --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2803x_CodeStartBranch.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

DSP2803x_usDelay.obj: C:/Users/Bartho\ Horn/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/source/DSP2803x_usDelay.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_6.2.5/bin/cl2000" -v28 -ml -mt --cla_support=cla0 -g --include_path="C:/ti/ccsv6/tools/compiler/c2000_6.2.5/include" --include_path="/packages/ti/xdais" --include_path="C:/Users/Bartho Horn/Documents/Github/BatteryBestuurStelsel/DSP2803x_headers/include" --include_path="C:/Users/Bartho Horn/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/include" --include_path="C:/Users/Bartho Horn/libs/math/IQmath/v160/include" --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2803x_usDelay.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_6.2.5/bin/cl2000" -v28 -ml -mt --cla_support=cla0 -g --include_path="C:/ti/ccsv6/tools/compiler/c2000_6.2.5/include" --include_path="/packages/ti/xdais" --include_path="C:/Users/Bartho Horn/Documents/Github/BatteryBestuurStelsel/DSP2803x_headers/include" --include_path="C:/Users/Bartho Horn/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/include" --include_path="C:/Users/Bartho Horn/libs/math/IQmath/v160/include" --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


