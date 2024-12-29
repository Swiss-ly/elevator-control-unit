/* ########################################################################

   Copyright (c) : 2015  Luis Claudio Gambôa Lopes

   ######################################################################## */

#include <xc.h>
#include "lcd.h"
#define _XTAL_FREQ 20000000  // tells compiler of XTAL frequency for delays

void lcd_wr(unsigned char val)
{
  LPORT=val;
}

void lcd_cmd(unsigned char val)
{
	LENA=1;
        lcd_wr(val);
        LDAT=0;
        __delay_ms(3);
        LENA=0;
        __delay_ms(3);
	LENA=1;
}
 
void lcd_dat(unsigned char val)
{
	LENA=1;
        lcd_wr(val);
        LDAT=1;
        __delay_ms(3);
        LENA=0;
        __delay_ms(3);
	LENA=1;
}

void lcd_init(void)
{
	TRISEbits.RE1 = 0;
	TRISEbits.RE2 = 0;
	TRISD = 0x00;
	LENA=0;
	LDAT=0;
  __delay_ms(20);
	LENA=1;
	
	lcd_cmd(L_CFG);
  __delay_ms(5);
	lcd_cmd(L_CFG);
  __delay_ms(1);
	lcd_cmd(L_CFG); //configura
	lcd_cmd(L_OFF);
	lcd_cmd(L_ON); //liga
	lcd_cmd(L_CLR); //limpa
	lcd_cmd(L_CFG); //configura
  lcd_cmd(L_L1);
}

void lcd_str(const char* str)
{
 unsigned char i=0;
  
 while(str[i] != 0 )
 {
   lcd_dat(str[i]);
   i++;
 }  
}
