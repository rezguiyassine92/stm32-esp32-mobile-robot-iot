/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_lcd.h"

int i;

uint8_t backlightState=1;


void Init_I2C(void)
{

//configure the AF
  RCC->AHB1ENR |= 1<<1;        //Clock for  GPIOB
  RCC->APB1ENR |= 1 <<21; //clock to I2C1

  GPIOB->AFR[0] |= (4<< 24);  //enable SCK to PB6
  GPIOB->AFR[0] |= (4<< 28);  //enable SDA to PB7

  GPIOB->MODER &= ~(3 << 12); //clear bits 12 & 13 (PB6)
  GPIOB->MODER |= 2 << 12; //MODER6[1:0] = 10 bin
  GPIOB->MODER &= ~(3 << 14); //clear bits 14 & 15 (PB7)
  GPIOB->MODER |= 2 << 14; //MODER9[1:0] = 10 bin

  GPIOB->OTYPER |= 1 <<6; //PB6 open drain
  GPIOB->OTYPER |= 1<<7; //PB7 open drain
  GPIOB->PUPDR &= ~(3 << 12);//clear bits 12 & 13 (PB6)
  GPIOB->PUPDR &= ~(3 << 14);//clear bits 14 & 15 (PB7)

  //Configure the I2C
	I2C1->CR2     = 0x0010; //16MHz must inferieur a 42MHZ
	I2C1->CCR     = 0x0050; //100kHz Bit rate
	I2C1->TRISE   = 0x0011; //1000 ns / 62.5 ns = 16 + 1
	
	I2C1->CR1    = 0x0001;      //enable peripheral
}



void lcd_Send(uint8_t data)
{

	 I2C1->CR1 |= I2C_CR1_START;//I2C_START;
	 while (!(I2C1->SR1 & 0x0001));

	 I2C1->DR =(0x20+LCD_ADDR) << 1;	
	 while (!(I2C1->SR1 & 0x0002)) {};  // wait for ADDRESS sent (ADDR=1)
	 int Status2= I2C1->SR2;    // read status to clear flag

	 while (!(I2C1->SR1 & 0x0080));  // wait for DR empty (TxE)
	 I2C1->DR = data;
	 I2C1->CR1 |= I2C_CR1_STOP;//I2C_stop(I2C1);
}

void lcd_Command(uint8_t com)
{
	uint8_t data = 0;
	data |= (backlightState & 0x01) << BL;
	data |= (com >>4)<<4;
	lcd_Send(data);
	data |= (1 << EN);
	lcd_Send(data);
	lcd_pause;
	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;
	data = 0;
	data |= (backlightState & 0x01) << BL;
	data |= (com&0x0F)<<4;
	lcd_Send(data);
	data |= (1 << EN);
	lcd_Send(data);
	lcd_pause;
	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;
}

void lcd_Data(uint8_t com)
{
	uint8_t data = 0;

	data |= (1 << EN);
	data |= (1 << RS);
	data |= (backlightState & 0x01) << BL;
	data |= (((com & 0x10) >> 4) << DB4);
	data |= (((com & 0x20) >> 5) << DB5);
	data |= (((com & 0x40) >> 6) << DB6);
	data |= (((com & 0x80) >> 7) << DB7);
	lcd_Send(data);
	
        lcd_pause;
	
        data &= ~(1 << EN);
        lcd_Send(data);
	lcd_pause;

	data = 0;

	data |= (1 << EN);
	data |= (1 << RS);
	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x01) >> 0) << DB4);
	data |= (((com & 0x02) >> 1) << DB5);
	data |= (((com & 0x04) >> 2) << DB6);
	data |= (((com & 0x08) >> 3) << DB7);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;
}

void lcd_Init(void)
{
	lcd_Command(0x33);
	lcd_pause;
	lcd_Command(0x32);
	lcd_Command(0x28);
	lcd_Command(0x08);
	lcd_Command(0x01);
	lcd_pause;
	lcd_Command(0x06);
	lcd_Command(0x0C);
}


void lcd_Backlight(uint8_t state)
{
	backlightState = (state & 0x01) << BL;
	lcd_Send(backlightState);
}

void lcd_PrintC(const uint8_t *str)
{
 	uint8_t i;
 	while (i=*str++)
 	{
    	  lcd_Data(i);
 	}
}

void lcd_Goto(uint8_t row, uint8_t col)
{
 #ifdef LCD_2004

 switch (row)
 {
		case 1:
			    lcd_Command(0x80 + col);
			    break;
		case 2:
			    lcd_Command(0x80 + col + 0x40);
			    break;
		case 3:
			    lcd_Command(0x80 + col + 0x14);
			    break;
		case 4:
			    lcd_Command(0x80 + col + 0x54);
			    break;
}
#endif
}