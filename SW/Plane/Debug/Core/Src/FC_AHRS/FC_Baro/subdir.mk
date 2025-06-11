################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FC_AHRS/FC_Baro/Baro.c 

OBJS += \
./Core/Src/FC_AHRS/FC_Baro/Baro.o 

C_DEPS += \
./Core/Src/FC_AHRS/FC_Baro/Baro.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/FC_AHRS/FC_Baro/%.o Core/Src/FC_AHRS/FC_Baro/%.su Core/Src/FC_AHRS/FC_Baro/%.cyclo: ../Core/Src/FC_AHRS/FC_Baro/%.c Core/Src/FC_AHRS/FC_Baro/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-FC_AHRS-2f-FC_Baro

clean-Core-2f-Src-2f-FC_AHRS-2f-FC_Baro:
	-$(RM) ./Core/Src/FC_AHRS/FC_Baro/Baro.cyclo ./Core/Src/FC_AHRS/FC_Baro/Baro.d ./Core/Src/FC_AHRS/FC_Baro/Baro.o ./Core/Src/FC_AHRS/FC_Baro/Baro.su

.PHONY: clean-Core-2f-Src-2f-FC_AHRS-2f-FC_Baro

