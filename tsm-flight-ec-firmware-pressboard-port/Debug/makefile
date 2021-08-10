################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/firmware-libraries/W25N01GV/src/subdir.mk
-include Core/firmware-libraries/ValveLibs/src/subdir.mk
-include Core/firmware-libraries/SerialComms/src/subdir.mk
-include Core/firmware-libraries/MAX31855/src/subdir.mk
-include Core/firmware-libraries/MAX11131/src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := tsm-flight-ec-firmware-pressboard-port
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
tsm-flight-ec-firmware-pressboard-port.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
tsm-flight-ec-firmware-pressboard-port.list \

OBJCOPY_BIN += \
tsm-flight-ec-firmware-pressboard-port.bin \


# All Target
all: main-build

# Main-build Target
main-build: tsm-flight-ec-firmware-pressboard-port.elf secondary-outputs

# Tool invocations
tsm-flight-ec-firmware-pressboard-port.elf: $(OBJS) $(USER_OBJS) C:\Users\pekin\Desktop\tsm-flight-ec-firmware\tsm-flight-ec-firmware-pressboard-port\STM32F446VETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "tsm-flight-ec-firmware-pressboard-port.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\pekin\Desktop\tsm-flight-ec-firmware\tsm-flight-ec-firmware-pressboard-port\STM32F446VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="tsm-flight-ec-firmware-pressboard-port.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

tsm-flight-ec-firmware-pressboard-port.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "tsm-flight-ec-firmware-pressboard-port.list"
	@echo 'Finished building: $@'
	@echo ' '

tsm-flight-ec-firmware-pressboard-port.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "tsm-flight-ec-firmware-pressboard-port.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_OUTPUT)$(OBJDUMP_LIST)$(EXECUTABLES)$(OBJS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS)$(OBJCOPY_BIN) tsm-flight-ec-firmware-pressboard-port.elf
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
