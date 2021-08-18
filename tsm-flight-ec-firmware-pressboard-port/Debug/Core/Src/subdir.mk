################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/autosequence.c \
../Core/Src/calibrations.c \
../Core/Src/globals.c \
../Core/Src/hardware.c \
../Core/Src/main.c \
../Core/Src/nonvolatile_memory.c \
../Core/Src/pack_calibration_defines.c \
../Core/Src/pack_cmd_defines.c \
../Core/Src/pack_telem_defines.c \
../Core/Src/sensor_voting.c \
../Core/Src/serial_data.c \
../Core/Src/status_flags.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tank_pressure_control.c \
../Core/Src/telem.c \
../Core/Src/threshold_detection.c \
../Core/Src/valves.c 

OBJS += \
./Core/Src/autosequence.o \
./Core/Src/calibrations.o \
./Core/Src/globals.o \
./Core/Src/hardware.o \
./Core/Src/main.o \
./Core/Src/nonvolatile_memory.o \
./Core/Src/pack_calibration_defines.o \
./Core/Src/pack_cmd_defines.o \
./Core/Src/pack_telem_defines.o \
./Core/Src/sensor_voting.o \
./Core/Src/serial_data.o \
./Core/Src/status_flags.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tank_pressure_control.o \
./Core/Src/telem.o \
./Core/Src/threshold_detection.o \
./Core/Src/valves.o 

C_DEPS += \
./Core/Src/autosequence.d \
./Core/Src/calibrations.d \
./Core/Src/globals.d \
./Core/Src/hardware.d \
./Core/Src/main.d \
./Core/Src/nonvolatile_memory.d \
./Core/Src/pack_calibration_defines.d \
./Core/Src/pack_cmd_defines.d \
./Core/Src/pack_telem_defines.d \
./Core/Src/sensor_voting.d \
./Core/Src/serial_data.d \
./Core/Src/status_flags.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tank_pressure_control.d \
./Core/Src/telem.d \
./Core/Src/threshold_detection.d \
./Core/Src/valves.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/autosequence.o: ../Core/Src/autosequence.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/autosequence.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/calibrations.o: ../Core/Src/calibrations.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/calibrations.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/globals.o: ../Core/Src/globals.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/globals.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/hardware.o: ../Core/Src/hardware.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/hardware.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/nonvolatile_memory.o: ../Core/Src/nonvolatile_memory.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/nonvolatile_memory.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/pack_calibration_defines.o: ../Core/Src/pack_calibration_defines.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/pack_calibration_defines.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/pack_cmd_defines.o: ../Core/Src/pack_cmd_defines.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/pack_cmd_defines.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/pack_telem_defines.o: ../Core/Src/pack_telem_defines.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/pack_telem_defines.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/sensor_voting.o: ../Core/Src/sensor_voting.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sensor_voting.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/serial_data.o: ../Core/Src/serial_data.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/serial_data.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/status_flags.o: ../Core/Src/status_flags.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/status_flags.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32f4xx_hal_msp.o: ../Core/Src/stm32f4xx_hal_msp.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32f4xx_it.o: ../Core/Src/stm32f4xx_it.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/sysmem.o: ../Core/Src/sysmem.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/system_stm32f4xx.o: ../Core/Src/system_stm32f4xx.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32f4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/tank_pressure_control.o: ../Core/Src/tank_pressure_control.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/tank_pressure_control.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/telem.o: ../Core/Src/telem.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/telem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/threshold_detection.o: ../Core/Src/threshold_detection.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/threshold_detection.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/valves.o: ../Core/Src/valves.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/firmware-libraries/MAX31855/inc -I../Core/firmware-libraries/MAX11131/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/valves.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

