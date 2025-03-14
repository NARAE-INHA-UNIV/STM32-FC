################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/FC_RC/dd.c 

OBJS += \
./Core/Inc/FC_RC/dd.o 

C_DEPS += \
./Core/Inc/FC_RC/dd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/FC_RC/%.o Core/Inc/FC_RC/%.su Core/Inc/FC_RC/%.cyclo: ../Core/Inc/FC_RC/%.c Core/Inc/FC_RC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-FC_RC

clean-Core-2f-Inc-2f-FC_RC:
	-$(RM) ./Core/Inc/FC_RC/dd.cyclo ./Core/Inc/FC_RC/dd.d ./Core/Inc/FC_RC/dd.o ./Core/Inc/FC_RC/dd.su

.PHONY: clean-Core-2f-Inc-2f-FC_RC

