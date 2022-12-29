# LCD_Drivers
LCD drivers implementation for STM32F407x-based processors.
* The project is developed in STM32Cube IDE.
* Here are the [GPIO Drivers](https://github.com/LalitK-Space/Driver_Development) GPIO drivers used in this project.

## Description
 General description of developed source and header files
  * LCD specific drivers are located in 'LCD_Drivers' folder.
 <br> `LCD_Drivers > LCD_Drivers` </br>
    * The 'LCD_Drivers' folder contains two sub-folders: *Inc* and *Src*.
    * The *Inc* folder contains the following:
        1. `lcd_drivers.h` (LCD-specific APIs)
    * The *Src* folder contains the following:
    1. `lcd_drivers.c` (LCD drivers implementation)
 
 * Device specific (GPIO) drivers are located in 'Device_Drivers' folder.
 <br> `LCD_Drivers > Device Drivers` </br>
    * The 'Device_Drivers' folder contains two sub-folders: *Inc* and *Src*.
    * The *Inc* folder contains the following:
        1. `stm32f407xx.h` (Device-specific header file)
        2. `stm32f407xx_gpio_drivers.h` (GPIO-specific APIs)
    * The *Src* folder contains the following:
    1. `stm32f407xx_gpio_drivers.c` (GPIO drivers implementation)



