################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GCS_MAVLink/GCS_Common.c 

OBJS += \
./Core/Src/GCS_MAVLink/GCS_Common.o 

C_DEPS += \
./Core/Src/GCS_MAVLink/GCS_Common.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/GCS_MAVLink/%.o Core/Src/GCS_MAVLink/%.su Core/Src/GCS_MAVLink/%.cyclo: ../Core/Src/GCS_MAVLink/%.c Core/Src/GCS_MAVLink/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-GCS_MAVLink

clean-Core-2f-Src-2f-GCS_MAVLink:
	-$(RM) ./Core/Src/GCS_MAVLink/GCS_Common.cyclo ./Core/Src/GCS_MAVLink/GCS_Common.d ./Core/Src/GCS_MAVLink/GCS_Common.o ./Core/Src/GCS_MAVLink/GCS_Common.su

.PHONY: clean-Core-2f-Src-2f-GCS_MAVLink

