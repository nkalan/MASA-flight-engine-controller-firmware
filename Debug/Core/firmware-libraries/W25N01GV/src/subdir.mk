################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/firmware-libraries/W25N01GV/src/W25M02GV.c \
../Core/firmware-libraries/W25N01GV/src/W25N01GV.c 

OBJS += \
./Core/firmware-libraries/W25N01GV/src/W25M02GV.o \
./Core/firmware-libraries/W25N01GV/src/W25N01GV.o 

C_DEPS += \
./Core/firmware-libraries/W25N01GV/src/W25M02GV.d \
./Core/firmware-libraries/W25N01GV/src/W25N01GV.d 


# Each subdirectory must supply rules for building sources it contributes
Core/firmware-libraries/W25N01GV/src/W25M02GV.o: ../Core/firmware-libraries/W25N01GV/src/W25M02GV.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/firmware-libraries/W25N01GV/src/W25M02GV.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/firmware-libraries/W25N01GV/src/W25N01GV.o: ../Core/firmware-libraries/W25N01GV/src/W25N01GV.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/firmware-libraries/W25N01GV/src/W25N01GV.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

