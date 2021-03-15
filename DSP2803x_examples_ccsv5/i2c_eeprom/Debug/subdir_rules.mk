################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
DSP2803x_CodeStartBranch.obj: C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_headers/include" --include_path="C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/include" --include_path="C:/Users/barth/OneDrive/libs/math/IQmath/v160/include" --define="_DEBUG" --define="LARGE_MODEL" -g --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --output_all_syms --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

DSP2803x_usDelay.obj: C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/source/DSP2803x_usDelay.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_headers/include" --include_path="C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/include" --include_path="C:/Users/barth/OneDrive/libs/math/IQmath/v160/include" --define="_DEBUG" --define="LARGE_MODEL" -g --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --output_all_syms --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_headers/include" --include_path="C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/include" --include_path="C:/Users/barth/OneDrive/libs/math/IQmath/v160/include" --define="_DEBUG" --define="LARGE_MODEL" -g --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --output_all_syms --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


