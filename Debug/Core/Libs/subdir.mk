################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libs/HD44780.c 

OBJS += \
./Core/Libs/HD44780.o 

C_DEPS += \
./Core/Libs/HD44780.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libs/%.o Core/Libs/%.su Core/Libs/%.cyclo: ../Core/Libs/%.c Core/Libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I/core/Libs/HD44780 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Libs

clean-Core-2f-Libs:
	-$(RM) ./Core/Libs/HD44780.cyclo ./Core/Libs/HD44780.d ./Core/Libs/HD44780.o ./Core/Libs/HD44780.su

.PHONY: clean-Core-2f-Libs

