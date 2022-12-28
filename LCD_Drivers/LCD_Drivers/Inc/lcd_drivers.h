/*
 * 									lcd_drivers.h
 *
 * This file contains all the LCD-related APIs supported by the driver.
 *
 */

#ifndef LCD_DRIVERS_INC_LCD_DRIVERS_H_
#define LCD_DRIVERS_INC_LCD_DRIVERS_H_

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_drivers.h"

/* -- Application Configurable Items  -- */
#define LCD_GPIO_PORT	              GPIOE		  // LCD is connected to GPIO port E		              
#define LCD_GPIO_RS		      GPIO_Pin_8	  // LCD RS is connected to Pin 8
#define LCD_GPIO_RW		      GPIO_Pin_9	  // LCD RW is connected to Pin 9
#define LCD_GPIO_E		      GPIO_Pin_10	  // LCD Enable is connected to Pin 10
#define LCD_GPIO_D4		      GPIO_Pin_11	  // LCD D4 is connected to Pin 11
#define LCD_GPIO_D5		      GPIO_Pin_12	  // LCD D5 is connected to Pin 12
#define LCD_GPIO_D6		      GPIO_Pin_13	  // LCD D6 is connected to Pin 12
#define LCD_GPIO_D7		      GPIO_Pin_14	  // LCD D7 is connected to Pin 14

/* -- (Some) LCD Commands -- */
#define LCD_CMD_4DL_2N_5X8F       0x28	// Data Length = 4, Lines = 2, Pixles = 5x8 dots
#define LCD_CMD_DISPLAY_CURSOR_ON 0x0E	// Display and Cursor ON
#define LCD_CMD_INCREMENT_CURSOR_NO_DISPLAY_SHIFT 	0x06	// Entry Mode Set
#define LCD_CMD_DISPLAY_CLEAR     0x01	// CLear Display
#define LCD_CMD_DISPLAY_RETURN_HOME 0x02	// Return Home


/* -- APIs Supported by LCD driver -- */
void LCD_Init(void);			                        // To initialize LCD
void LCD_SendCommands(uint8_t command);	          // To send Commands
void LCD_SendChar(uint8_t data);	                // To send char
void LCD_SendString(char *data);	                // To send string
void LCD_ClearDisplay(void);		                  // To clear Display
void LCD_ReturnHome(void);		                    // Display Return Home
void LCD_SetCursor(uint8_t row, uint8_t column);	// To specify cursor location (Row and Column)



#endif /* LCD_DRIVERS_INC_LCD_DRIVERS_H_ */
