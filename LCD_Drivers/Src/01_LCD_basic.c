


#include "lcd_drivers.h"
#include <stdio.h>

int main(void)
{
	LCD_Init();

	LCD_SendString(" ...TESTING...  ");
	LCD_SetCursor(2, 1);
	LCD_SendString("www.lalitk.space");


	while(1);

	return 0;
}


