################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Device_Drivers/Src/stm32f407xx_gpio_drivers.c 

OBJS += \
./Device_Drivers/Src/stm32f407xx_gpio_drivers.o 

C_DEPS += \
./Device_Drivers/Src/stm32f407xx_gpio_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
Device_Drivers/Src/%.o Device_Drivers/Src/%.su: ../Device_Drivers/Src/%.c Device_Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/Resources/Embedded_Systems/LCD_Drivers/LCD_Drivers/LCD_Drivers/Device_Drivers/Inc" -I"D:/Resources/Embedded_Systems/LCD_Drivers/LCD_Drivers/LCD_Drivers/LCD_Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Device_Drivers-2f-Src

clean-Device_Drivers-2f-Src:
	-$(RM) ./Device_Drivers/Src/stm32f407xx_gpio_drivers.d ./Device_Drivers/Src/stm32f407xx_gpio_drivers.o ./Device_Drivers/Src/stm32f407xx_gpio_drivers.su

.PHONY: clean-Device_Drivers-2f-Src

