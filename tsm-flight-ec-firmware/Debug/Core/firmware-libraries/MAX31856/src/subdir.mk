################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/firmware-libraries/MAX31856/src/MAX31856.c 

OBJS += \
./Core/firmware-libraries/MAX31856/src/MAX31856.o 

C_DEPS += \
./Core/firmware-libraries/MAX31856/src/MAX31856.d 


# Each subdirectory must supply rules for building sources it contributes
Core/firmware-libraries/MAX31856/src/%.o: ../Core/firmware-libraries/MAX31856/src/%.c Core/firmware-libraries/MAX31856/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Core/firmware-libraries/MAX31856/inc -I../Core/firmware-libraries/ValveLibs/inc -I../Core/firmware-libraries/MAX11128/inc -I../Core/firmware-libraries/SerialComms/inc -I../Core/firmware-libraries/L6470/inc -I../Core/firmware-libraries/W25N01GV/inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

