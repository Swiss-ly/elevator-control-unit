/* ########################################################################

   Copyright (c) : 2015  Luis Claudio Gamb�a Lopes

    ######################################################################## */

#define LENA  PORTEbits.RE1
#define LDAT  PORTEbits.RE2
#define LPORT PORTD


#define L_ON	0x0F     // turn display on
#define L_OFF	0x08     // turn display off
#define L_CLR	0x01     // clear display and home cursor
#define L_L1	0x80     // move to line 1 first character (no clear)
#define L_L2	0xC0     // move to line 2 first character (no clear)
#define L_CR	0x0F     // blinking cursor on (display on)		
#define L_NCR	0x0C     // turn cursor off
#define L_LFT	0x10     // move cursor left
#define L_RGT	0x14     // move cursor right

#define L_CFG   0x38     // code for configuring 4 bit mode and other settings

void lcd_init(void);
void lcd_cmd(unsigned char val); 
void lcd_dat(unsigned char val);
void lcd_str(const char* str);

