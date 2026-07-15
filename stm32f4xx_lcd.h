/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_LCD_H
#define __STM32F4xx_LCD_H



/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

extern void Delay(int ncount1);


#define lcd_pause	Delay(120750)



#define PCF_P0	0
#define PCF_P1	1
#define PCF_P2	2
#define PCF_P3	3
#define PCF_P4	4
#define PCF_P5	5
#define PCF_P6	6
#define PCF_P7	7


#define DB4		PCF_P4
#define DB5		PCF_P5
#define DB6		PCF_P6
#define DB7		PCF_P7
#define EN		PCF_P2
#define RW		PCF_P1
#define RS		PCF_P0
#define BL		PCF_P3

#define LCD_ADDR	0x06


#define LCD_2004




void lcd_Send(uint8_t data);
void lcd_Command(uint8_t com);
void lcd_Init(void);
void lcd_Backlight(uint8_t state);
void lcd_Goto(uint8_t row, uint8_t col);
void lcd_PrintC(const uint8_t *str);

void Init_I2C(void);


#endif /* __STM32F4xx_CAN_H */

