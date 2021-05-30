################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/firmware-libraries/L6470/src/L6470.c 

OBJS += \
./Core/firmware-libraries/L6470/src/L6470.o 

C_DEPS += \
./Core/firmware-libraries/L6470/src/L6470.d 


# Each subdirectory must supply rules for building sources it contributes
Core/firmware-libraries/L6470/src/L6470.o: ../Core/firmware-libraries/L6470/src/L6470.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/firmware-libraries/L6470/src/L6470.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

