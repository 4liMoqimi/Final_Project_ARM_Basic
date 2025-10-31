################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD16X2/LCD16X2.c \
../LCD16X2/LCD16X2_cfg.c 

OBJS += \
./LCD16X2/LCD16X2.o \
./LCD16X2/LCD16X2_cfg.o 

C_DEPS += \
./LCD16X2/LCD16X2.d \
./LCD16X2/LCD16X2_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
LCD16X2/%.o LCD16X2/%.su LCD16X2/%.cyclo: ../LCD16X2/%.c LCD16X2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD16X2

clean-LCD16X2:
	-$(RM) ./LCD16X2/LCD16X2.cyclo ./LCD16X2/LCD16X2.d ./LCD16X2/LCD16X2.o ./LCD16X2/LCD16X2.su ./LCD16X2/LCD16X2_cfg.cyclo ./LCD16X2/LCD16X2_cfg.d ./LCD16X2/LCD16X2_cfg.o ./LCD16X2/LCD16X2_cfg.su

.PHONY: clean-LCD16X2

