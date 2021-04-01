################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./DSP2803x_CodeStartBranch.obj" \
"./DSP2803x_usDelay.obj" \
"./Main.obj" \
"./Source/DSP2803x_Adc.obj" \
"./Source/DSP2803x_CpuTimers.obj" \
"./Source/DSP2803x_DefaultIsr.obj" \
"./Source/DSP2803x_ECan.obj" \
"./Source/DSP2803x_GlobalVariableDefs.obj" \
"./Source/DSP2803x_I2C.obj" \
"./Source/DSP2803x_PieCtrl.obj" \
"./Source/DSP2803x_PieVect.obj" \
"./Source/DSP2803x_Spi.obj" \
"./Source/DSP2803x_SysCtrl.obj" \
"./User/User_ADC.obj" \
"./User/User_BQ.obj" \
"./User/User_CAN.obj" \
"./User/User_Functions.obj" \
"./User/User_Globals.obj" \
"./User/User_I2C.obj" \
"./User/User_Interrupts.obj" \
"./User/User_Queue.obj" \
"C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_headers/cmd/DSP2803x_Headers_nonBIOS.cmd" \
"../F28035.cmd" \
"../IQmath.lib" \
$(GEN_CMDS__FLAG) \
-lrts2800_ml.lib \
-lIQmath.lib \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Source/subdir_vars.mk
-include User/subdir_vars.mk
-include subdir_rules.mk
-include Source/subdir_rules.mk
-include User/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Example_2803xI2C_eeprom.out \

EXE_OUTPUTS__QUOTED += \
"Example_2803xI2C_eeprom.out" \

BIN_OUTPUTS += \
Example_2803xI2C_eeprom.hex \

BIN_OUTPUTS__QUOTED += \
"Example_2803xI2C_eeprom.hex" \


# All Target
all: Example_2803xI2C_eeprom.out

# Tool invocations
Example_2803xI2C_eeprom.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --define=_DEBUG --define=LARGE_MODEL -g --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --output_all_syms -z -m"Example_2803xI2C_eeprom.map" --stack_size=0x780 --warn_sections -i"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/lib" -i"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/include" -i"C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_examples_ccsv5/i2c_eeprom" -i"C:/Users/barth/OneDrive/Documents/Github/BatteryBestuurStelsel/DSP2803x_common/lib" -i"C:/Users/barth/OneDrive/libs/math/IQmath/v160/lib" --reread_libs --issue_remarks --verbose_diagnostics --xml_link_info="Example_2803xI2C_eeprom_linkInfo.xml" --entry_point=code_start --rom_model -o "Example_2803xI2C_eeprom.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

Example_2803xI2C_eeprom.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: C2000 Hex Utility'
	"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/bin/hex2000" --diag_wrap=off -o "Example_2803xI2C_eeprom.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "DSP2803x_CodeStartBranch.obj" "DSP2803x_usDelay.obj" "Main.obj" "Source\DSP2803x_Adc.obj" "Source\DSP2803x_CpuTimers.obj" "Source\DSP2803x_DefaultIsr.obj" "Source\DSP2803x_ECan.obj" "Source\DSP2803x_GlobalVariableDefs.obj" "Source\DSP2803x_I2C.obj" "Source\DSP2803x_PieCtrl.obj" "Source\DSP2803x_PieVect.obj" "Source\DSP2803x_Spi.obj" "Source\DSP2803x_SysCtrl.obj" "User\User_ADC.obj" "User\User_BQ.obj" "User\User_CAN.obj" "User\User_Functions.obj" "User\User_Globals.obj" "User\User_I2C.obj" "User\User_Interrupts.obj" "User\User_Queue.obj" 
	-$(RM) "Main.d" "Source\DSP2803x_Adc.d" "Source\DSP2803x_CpuTimers.d" "Source\DSP2803x_DefaultIsr.d" "Source\DSP2803x_ECan.d" "Source\DSP2803x_GlobalVariableDefs.d" "Source\DSP2803x_I2C.d" "Source\DSP2803x_PieCtrl.d" "Source\DSP2803x_PieVect.d" "Source\DSP2803x_Spi.d" "Source\DSP2803x_SysCtrl.d" "User\User_ADC.d" "User\User_BQ.d" "User\User_CAN.d" "User\User_Functions.d" "User\User_Globals.d" "User\User_I2C.d" "User\User_Interrupts.d" "User\User_Queue.d" 
	-$(RM) "DSP2803x_CodeStartBranch.d" "DSP2803x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

