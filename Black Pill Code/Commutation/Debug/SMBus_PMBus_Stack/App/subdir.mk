################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.c 

OBJS += \
./SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.o 

C_DEPS += \
./SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.d 


# Each subdirectory must supply rules for building sources it contributes
SMBus_PMBus_Stack/App/%.o SMBus_PMBus_Stack/App/%.su SMBus_PMBus_Stack/App/%.cyclo: ../SMBus_PMBus_Stack/App/%.c SMBus_PMBus_Stack/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../SMBus_PMBus_Stack/Target -I../Middlewares/ST/STM32_SMBus_Stack/inc -I../SMBus_PMBus_Stack/App -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SMBus_PMBus_Stack-2f-App

clean-SMBus_PMBus_Stack-2f-App:
	-$(RM) ./SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.cyclo ./SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.d ./SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.o ./SMBus_PMBus_Stack/App/app_X-CUBE-SMBUS.su

.PHONY: clean-SMBus_PMBus_Stack-2f-App

