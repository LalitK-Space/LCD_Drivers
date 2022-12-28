################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD_Drivers/Src/lcd_drivers.c 

OBJS += \
./LCD_Drivers/Src/lcd_drivers.o 

C_DEPS += \
./LCD_Drivers/Src/lcd_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
LCD_Drivers/Src/%.o LCD_Drivers/Src/%.su: ../LCD_Drivers/Src/%.c LCD_Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/Resources/Embedded_Systems/LCD_Drivers/LCD_Drivers/LCD_Drivers/Device_Drivers/Inc" -I"D:/Resources/Embedded_Systems/LCD_Drivers/LCD_Drivers/LCD_Drivers/LCD_Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD_Drivers-2f-Src

clean-LCD_Drivers-2f-Src:
	-$(RM) ./LCD_Drivers/Src/lcd_drivers.d ./LCD_Drivers/Src/lcd_drivers.o ./LCD_Drivers/Src/lcd_drivers.su

.PHONY: clean-LCD_Drivers-2f-Src

