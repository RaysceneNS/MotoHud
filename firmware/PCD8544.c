#include "PCD8544.h"

void LcdInitialise(void)
{
	CLEARBIT(PORT_LCD_RST, PIN_LCD_RST); //RST LOW
	// delay(1);
	SETBIT(PORT_LCD_RST, PIN_LCD_RST);  //RST HIGH

	LcdWrite(LCD_CMD, PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );  
	LcdWrite(LCD_CMD, PCD8544_SETVOP | 0x3F);  // Set LCD VOP (Contrast). 
	LcdWrite(LCD_CMD, PCD8544_SETTEMP | 0x00 );  // Set Temp coefficient. 
	LcdWrite(LCD_CMD, PCD8544_SETBIAS | 0x04 );  // LCD bias mode 1:48.
	LcdWrite(LCD_CMD, PCD8544_FUNCTIONSET);//Function Set:0 0 1 0 0 PD V H=0010 0000;PD=0,V=0,H=0;
	LcdWrite(LCD_CMD, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

/*
Print an ascii character at the current screen location font is 7x5
*/
void LcdCharacter(uint8_t x, uint8_t y, char character)
{
	LcdGotoXY(x,y);
	for (int index = 0; index < 5; index++)
	{
		LcdWrite(LCD_DATA, pgm_read_byte(&ASCII[character - 0x20][index]));
	}
}

/*
Draw a medium digit at the current position
*/
void LcdMediumDigit(uint8_t x, uint8_t y, uint8_t digit)
{
	uint8_t idx;
	if(digit == 0x0A)
	{
		idx = 0;
	}
	else
	{
		idx = digit;
	}
	
	LcdGotoXY(x,y);
	for (int byt = 0; byt < 12; byt++)
	{
		LcdWrite(LCD_DATA, pgm_read_byte(&MEDIUMNUMBERS[idx][byt]));
	}
	LcdGotoXY(x,y+1);
	for (int byt = 0; byt < 12; byt++)
	{
		LcdWrite(LCD_DATA, pgm_read_byte(&MEDIUMNUMBERS[idx][byt+12]));
	}
}

/*
Draw a gliff at the current position
*/
void LcdGliff(uint8_t x, uint8_t y, uint8_t width, const uint8_t *BMP)
{
	LcdGotoXY(x, y);
	for (int index = 0; index < width; index++)
	{
		LcdWrite(LCD_DATA, pgm_read_byte(&BMP[index]));
	}
}

/*
Clears the screen by writing zeros to all locations
*/
void LcdClear(void)
{
	LcdGotoXY(0,0);
	for (int index = 0; index < 504; index++)
	{
		LcdWrite(LCD_DATA, 0x00);
	}
}

void LcdClearBlock(uint8_t x, uint8_t y, uint8_t cols, uint8_t rows)
{
	for(uint8_t i=0; i<rows;i++)
	{
		LcdGotoXY(x,y+i);
		for (int j= 0; j< cols; j++)
		{
			LcdWrite(LCD_DATA, 0x00);
		}
	}
}

void LcdString(char *characters)
{
	while (*characters)
	{
		LcdWrite(LCD_DATA, 0x00);
		uint8_t c = (*characters++) - 0x20;
		for (int index = 0; index < 5; index++)
		{
			LcdWrite(LCD_DATA, pgm_read_byte(&ASCII[c][index]));
		}
		LcdWrite(LCD_DATA, 0x00);
	}
}

void LcdWrite(uint8_t dc, uint8_t data)
{
	//Chip Enable:Active LOW
	CLEARBIT(PORT_LCD_SCE, PIN_LCD_SCE);
	
	// D/C=0:the current data byte is interpreted as command byte
	if(dc == 0)
	{
		CLEARBIT(PORT_LCD_DC, PIN_LCD_DC);
	}
	else
	{
		SETBIT(PORT_LCD_DC, PIN_LCD_DC);
	}
		
	for(uint8_t i=0; i<8; i++)
	{
		if(data & 0x80) //1000 0000
		{
			SETBIT(PORT_LCD_DIN, PIN_LCD_DIN);
		}
		else
		{
			CLEARBIT(PORT_LCD_DIN, PIN_LCD_DIN);
		}
		
		CLEARBIT(PORT_LCD_CLK, PIN_LCD_CLK);
		data = data << 1;
		SETBIT(PORT_LCD_CLK, PIN_LCD_CLK);
	}
	
	// Chip disable
	SETBIT(PORT_LCD_SCE, PIN_LCD_SCE);
}

// gotoXY routine to position cursor
// x - range: 0 to 84
// y - range: 0 to 5
void LcdGotoXY(uint8_t x, uint8_t y)
{
	LcdWrite(LCD_CMD, 0x80 | x);  // Column.
	LcdWrite(LCD_CMD, 0x40 | y);  // Row.
}

/*
Display the bitmap at location x,y

Bitmaps are RLE encoded
*/
void LcdBitmap(const uint8_t *BMP)
{
	uint8_t col=0, row=0;
	uint8_t data=0,count=0;
	uint16_t i=0;
	uint8_t idx = 0;
	
	for (row=0; row < 5; row++)
	{	
		LcdGotoXY(0, row);
		
		for (col=0; col < 40; col++)
		{
			if(count==0)
			{
				data = pgm_read_byte(&BMP[i++]);
				if(data==pgm_read_byte(&BMP[i++]))
				{
					count = pgm_read_byte(&BMP[i++]);
				}
				else
				{
					count = 1;
					i--;
				}
			}
			count--;
			
			screen_buff[idx] |= data;
			idx++;
		}
	}
}

void LcdBitBlt()
{
	uint8_t idx = 0;
	for (uint8_t row=0; row < 5; row++)
	{
		LcdGotoXY(0, row);
		for (uint8_t col=0; col < 40; col++)
		{
			LcdWrite(LCD_DATA, screen_buff[idx] );
			idx++;
		}
	}
}

