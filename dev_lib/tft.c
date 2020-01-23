/*
 * tft.c
 *
 *  Created on: Sep 24, 2019
 *      Author: Sigit
 */

#include "tft.h"
#include "delay.h"
#include "glcdfont.c"

static const uint16_t _regValues[]= {
		0x00e5,0x8000,
		0x0000,0x0001,

		0x0001,0x0100,
		0x0002,0x0700,
		0x0003,0x1030,
		0x0004,0x0000,
		0x0008,0x0202,
		0x0009,0x0000,
		0x000a,0x0000,
		0x000c,0x0000,
		0x000d,0x0000,
		0x000f,0x0000,
//*********************************************Power On
		0x0010,0x0000,
		0x0011,0x0000,
		0x0012,0x0000,
		0x0013,0x0000,

		0x0010,0x17b0,
		0x0011,0x0037,

		0x0012,0x0138,

		0x0013,0x1700,
		0x0029,0x000d,

		0x0020,0x0000,
		0x0021,0x0000,
//*********************************************Set gamma
		0x0030,0x0001,
		0x0031,0x0606,
		0x0032,0x0304,
		0x0033,0x0202,
		0x0034,0x0202,
		0x0035,0x0103,
		0x0036,0x011d,
		0x0037,0x0404,
		0x0038,0x0404,
		0x0039,0x0404,
		0x003c,0x0700,
		0x003d,0x0a1f,
//**********************************************Set Gram aera
		0x0050,0x0000,
		0x0051,0x00ef,
		0x0052,0x0000,
		0x0053,0x013f,
		0x0060,0x2700,
		0x0061,0x0001,
		0x006a,0x0000,
//*********************************************Paratial display
		0x0090,0x0010,
		0x0092,0x0000,
		0x0093,0x0003,
		0x0095,0x0101,
		0x0097,0x0000,
		0x0098,0x0000,
//******************************************** Plan Control
		0x0007,0x0021,

		0x0007,0x0031,

		0x0007,0x0173,

//		LLCD_WRITE_CMD(0x0022,

  // Display On

};

inline void tft_pin_init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable TFT_PORT Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure the GPIO pins */
	GPIO_InitStructure.GPIO_Pin = (TFT_CD_PIN | TFT_CS_PIN | TFT_RST_PIN
			| TFT_WR_PIN | TFT_RD_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(TFT_PORT, &GPIO_InitStructure);
}

inline void tft_write8(uint8_t d) {
	DATAPORT2 = (DATAPORT2 & DATA1_MASK) | (d & DATA2_MASK);
	DATAPORT1 = (DATAPORT1 & DATA2_MASK) | (d & DATA1_MASK); // top 6 bits
}

inline void tft_setWriteDir(void) {
	DATADDR2 |= DATA2_MASK;
	DATADDR1 |= DATA1_MASK;
}

// the C/D pin is low during write
void tft_writeCommand(uint16_t cmd) {
	cs_pin_low();
	cd_pin_low();
	rd_pin_high();
	wr_pin_high();

	tft_setWriteDir();
	tft_write8(cmd >> 8);

	wr_pin_low();
	wr_pin_high();

	tft_write8(cmd);

	wr_pin_low();
	wr_pin_high();
	cs_pin_high();
}

// the C/D pin is high during write
void tft_writeData(uint16_t data) {
	cs_pin_low();
	cd_pin_high();
	rd_pin_high();
	wr_pin_high();

	tft_setWriteDir();
	tft_write8(data >> 8);

	wr_pin_low();
	wr_pin_high();

	tft_write8(data);

	wr_pin_low();
	wr_pin_high();
	cs_pin_high();
}

// this is a 'speed up' version, with no direction setting, or pin initialization
// not for external usage, but it does speed up stuff like a screen fill
inline void tft_writeData_unsafe(uint16_t data) {
	tft_write8(data >> 8);

	wr_pin_low();
	wr_pin_high();

	tft_write8(data);

	wr_pin_low();
	wr_pin_high();
}

void tft_writeRegister(uint16_t addr, uint16_t data) {
	tft_writeCommand(addr);
	tft_writeData(data);
}

void tft_goTo(int x, int y) {
	tft_writeRegister(0x0020, x);     // GRAM Address Set (Horizontal Address) (R20h)
	tft_writeRegister(0x0021, y);     // GRAM Address Set (Vertical Address) (R21h)
	tft_writeCommand(0x0022);            // Write Data to GRAM (R22h)
}

void tft_goHome(void) {
	tft_goTo(0,0);
}

void tft_begin() {
	rotation = 0;
	_width = TFTWIDTH;
	_height = TFTHEIGHT;

	// disable the LCD
	tft_pin_init();
	cs_pin_high() ; cd_pin_high() ; wr_pin_high() ; rd_pin_high() ; reset_pin_high();

	cursor_y = cursor_x = 0;
	textsize = 1;
	textcolor = 0xFFFF;
}

void tft_reset(void) {
	reset_pin_low();
	delay_ms(2);
	reset_pin_high();

	// resync
	tft_writeData(0);
	tft_writeData(0);
	tft_writeData(0);
	tft_writeData(0);
}

static inline void ddelay_ms(uint16_t count) {
	while(count--) {
		delay_ms(1);

	}
}
void tft_initDisplay(void) {
	uint16_t a, d;

	tft_reset();

	for (uint8_t i = 0; i < sizeof(_regValues) / 4; i++) {
		a = pgm_read_word(_regValues + i*2);
		d = pgm_read_word(_regValues + i*2 + 1);

		if (a == 0xFF) {
			ddelay_ms(d);
			} else {
			tft_writeRegister(a, d);
		}
	}
}

void tft_fillScreen(uint16_t color) {
	tft_goHome();
	uint32_t i;

	i = 320;
	i *= 240;

	cs_pin_low();
	cd_pin_high();
	rd_pin_high();
	wr_pin_high();

	tft_setWriteDir();
	while (i--) {
		tft_writeData_unsafe(color);
	}

	cs_pin_high();
}

void tft_setCursor(uint16_t x, uint16_t y) {
	cursor_x = x;
	cursor_y = y;
}

void tft_setTextSize(uint8_t s) {
	textsize = s;
}

void tft_setTextColor(uint16_t c) {
	textcolor = c;
}

void tft_drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	// check rotation, move pixel around if necessary
	switch (rotation) {
		case 1:
		swap(x, y);
		x = TFTWIDTH - x - 1;
		break;
		case 2:
		x = TFTWIDTH - x - 1;
		y = TFTHEIGHT - y - 1;
		break;
		case 3:
		swap(x, y);
		y = TFTHEIGHT - y - 1;
		break;
	}

	if ((x >= TFTWIDTH) || (y >= TFTHEIGHT)) return;
	tft_writeRegister(TFTLCD_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address) (R20h)
	tft_writeRegister(TFTLCD_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address) (R21h)
	tft_writeCommand(TFTLCD_RW_GRAM);  // Write Data to GRAM (R22h)
	tft_writeData(color);
}

void tft_drawFastLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color, uint8_t rotflag) {
	uint16_t newentrymod;

	switch (rotation) {
		case 0:
		if (rotflag)
		newentrymod = 0x1028;   // we want a 'vertical line'
		else
		newentrymod = 0x1030;   // we want a 'horizontal line'
		break;
		case 1:
		swap(x, y);
		// first up fix the X
		x = TFTWIDTH - x - 1;
		if (rotflag)
		newentrymod = 0x1000;   // we want a 'vertical line'
		else
		newentrymod = 0x1028;   // we want a 'horizontal line'
		break;
		case 2:
		x = TFTWIDTH - x - 1;
		y = TFTHEIGHT - y - 1;
		if (rotflag)
		newentrymod = 0x1008;   // we want a 'vertical line'
		else
		newentrymod = 0x1020;   // we want a 'horizontal line'
		break;
		case 3:
		swap(x,y);
		y = TFTHEIGHT - y - 1;
		if (rotflag)
		newentrymod = 0x1030;   // we want a 'vertical line'
		else
		newentrymod = 0x1008;   // we want a 'horizontal line'
		break;
	}

	tft_writeRegister(TFTLCD_ENTRY_MOD, newentrymod);

	tft_writeRegister(TFTLCD_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address) (R20h)
	tft_writeRegister(TFTLCD_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address) (R21h)
	tft_writeCommand(TFTLCD_RW_GRAM);  // Write Data to GRAM (R22h)


	cs_pin_low();
	cd_pin_high();
	rd_pin_high();
	wr_pin_high();

	tft_setWriteDir();
	while (length--) {
		tft_writeData_unsafe(color);
	}

	// set back to default
	cs_pin_high();
	tft_writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
}

void tft_drawVerticalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
	if (x >= _width) return;
	tft_drawFastLine(x,y,length,color,1);
}

void tft_drawHorizontalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color) {
	if (y >= _height) return;
	tft_drawFastLine(x,y,length,color,0);
}

void tft_fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fillcolor) {
	// smarter version
	while (h--)
	tft_drawHorizontalLine(x, y++, w, fillcolor);
}

void tft_drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint8_t size) {
	for (uint8_t i =0; i<5; i++ ) {
		uint8_t line = pgm_read_byte(font+(c*5)+i);
		for (uint8_t j = 0; j<8; j++) {
			if (line & 0x1) {
				if (size == 1) // default size
				tft_drawPixel(x+i, y+j, color);
				else {  // big size
					tft_fillRect(x+i*size, y+j*size, size, size, color);
				}
			}
			line >>= 1;
		}
	}
}

void tft_write(uint8_t c) {
	if (c == '\n') {
		cursor_y += textsize*8;
		cursor_x = 0;
		} else if (c == '\r') {
		// skip em
		} else {
		tft_drawChar(cursor_x, cursor_y, c, textcolor, textsize);
		cursor_x += textsize*6;
	}
}

void tft_print(const char *str) {
	while(*str){
		tft_write(*str);
		str++;
	}
}

void tft_println(const char *str) {
	while(*str){
		tft_write(*str);
		str++;
	}
	tft_write('\r');
	tft_write('\n');
}

void tft_setRotation(uint8_t x) {
	tft_writeRegister(TFTLCD_ENTRY_MOD, 0x1030);

	x %= 4;  // cant be higher than 3
	rotation = x;
	switch (x) {
		case 0:
		_width = TFTWIDTH;
		_height = TFTHEIGHT;
		break;
		case 1:
		_width = TFTHEIGHT;
		_height = TFTWIDTH;
		break;
		case 2:
		_width = TFTWIDTH;
		_height = TFTHEIGHT;
		break;
		case 3:
		_width = TFTHEIGHT;
		_height = TFTWIDTH;
		break;
	}
}
// draw a rectangle
void tft_drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	// smarter version
	tft_drawHorizontalLine(x, y, w, color);
	tft_drawHorizontalLine(x, y+h-1, w, color);
	tft_drawVerticalLine(x, y, h, color);
	tft_drawVerticalLine(x+w-1, y, h, color);
}

void tft_draw565CutoutBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* data) {
	uint16_t col;
	uint16_t tx, ty, tc;
	for (ty = 0; ty < h; ty++) {
		for (tx = 0; tx < w; tx++) {
			tc = (ty * w) + tx;
			col = (uint16_t) (data[tc]);
			if(data[tc] != 0x0000){
				tft_drawPixel(x + tx, y + ty, col);
			}
		}
	}
}
/************************************************************************************

//void tft_drawString(uint16_t x, uint16_t y, char *c, uint16_t color, uint8_t size) {
	//while (c[0] != 0) {
		//tft_drawChar(x, y, c[0], color, size);
		//x += size*6;
		//c++;
	//}
//}
//
uint16_t tft_width(void) {
  return _width;
}
uint16_t tft_height(void) {
  return _height;
}
// draw a triangle!
void tft_drawTriangle(uint16_t x0, uint16_t y0,
			  uint16_t x1, uint16_t y1,
			  uint16_t x2, uint16_t y2, uint16_t color)
{
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}

void tft_fillTriangle ( int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color)
{
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  int32_t dx1, dx2, dx3; // Interpolation deltas
  int32_t sx1, sx2, sy; // Scanline co-ordinates

  sx2=(int32_t)x0 * (int32_t)1000; // Use fixed point math for x axis values
  sx1 = sx2;
  sy=y0;

  // Calculate interpolation deltas
  if (y1-y0 > 0) dx1=((x1-x0)*1000)/(y1-y0);
    else dx1=0;
  if (y2-y0 > 0) dx2=((x2-x0)*1000)/(y2-y0);
    else dx2=0;
  if (y2-y1 > 0) dx3=((x2-x1)*1000)/(y2-y1);
    else dx3=0;

  // Render scanlines (horizontal lines are the fastest rendering method)
  if (dx1 > dx2)
  {
    for(; sy<=y1; sy++, sx1+=dx2, sx2+=dx1)
    {
      drawHorizontalLine(sx1/1000, sy, (sx2-sx1)/1000, color);
    }
    sx2 = x1*1000;
    sy = y1;
    for(; sy<=y2; sy++, sx1+=dx2, sx2+=dx3)
    {
      drawHorizontalLine(sx1/1000, sy, (sx2-sx1)/1000, color);
    }
  }
  else
  {
    for(; sy<=y1; sy++, sx1+=dx1, sx2+=dx2)
    {
      drawHorizontalLine(sx1/1000, sy, (sx2-sx1)/1000, color);
    }
    sx1 = x1*1000;
    sy = y1;
    for(; sy<=y2; sy++, sx1+=dx3, sx2+=dx2)
    {
      drawHorizontalLine(sx1/1000, sy, (sx2-sx1)/1000, color);
    }
  }
}
uint16_t tft_Color565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c;
  c = r >> 3;
  c <<= 6;
  c |= g >> 2;
  c <<= 5;
  c |= b >> 3;

  return c;
}
// draw a rounded rectangle
void tft_drawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r,
			   uint16_t color) {
  // smarter version
  drawHorizontalLine(x+r, y, w-2*r, color);
  drawHorizontalLine(x+r, y+h-1, w-2*r, color);
  drawVerticalLine(x, y+r, h-2*r, color);
  drawVerticalLine(x+w-1, y+r, h-2*r, color);
  // draw four corners
  drawCircleHelper(x+r, y+r, r, 1, color);
  drawCircleHelper(x+w-r-1, y+r, r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r, y+h-r-1, r, 8, color);
}
// fill a rounded rectangle
void tft_fillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r,
			   uint16_t color) {
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r, y+r, r, 2, h-2*r-1, color);
}
// fill a circle
void tft_fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
  writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
  drawVerticalLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}
// used to do circles and roundrects!
void tft_fillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t delta,
			uint16_t color) {

  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    if (cornername & 0x1) {
      drawVerticalLine(x0+x, y0-y, 2*y+1+delta, color);
      drawVerticalLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawVerticalLine(x0-x, y0-y, 2*y+1+delta, color);
      drawVerticalLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}
// draw a circle outline

void tft_drawCircle(uint16_t x0, uint16_t y0, uint16_t r,
			uint16_t color) {
  drawPixel(x0, y0+r, color);
  drawPixel(x0, y0-r, color);
  drawPixel(x0+r, y0, color);
  drawPixel(x0-r, y0, color);

  drawCircleHelper(x0, y0, r, 0xF, color);
}
void tft_drawCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername,
			uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;


  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}
// bresenham's algorithm - thx wikpedia
void tft_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
		      uint16_t color) {
  // if you're in rotation 1 or 3, we need to swap the X and Y's

  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  //dy = abs(y1 - y0);
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

uint8_t tft_getRotation(void) {
  return rotation;
}

inline void tft_setReadDir(void) {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328) || (__AVR_ATmega8__)
  DATADDR2 &= ~DATA2_MASK;
  DATADDR1 &= ~DATA1_MASK;
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

  #ifdef USE_ADAFRUIT_SHIELD_PINOUT
  DDRH &= ~0x78;
  DDRB &= ~0xB0;
  DDRG &= ~_BV(5);
  #else
  MEGA_DATADDR = 0;
  #endif
#else
  #error "No pins defined!"
#endif
}

inline uint8_t tft_read8(void) {
 uint8_t d;
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328) || (__AVR_ATmega8__)

 d = DATAPIN1 & DATA1_MASK;
 d |= DATAPIN2 & DATA2_MASK;

#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__)  || defined(__AVR_ATmega1280__)

#ifdef USE_ADAFRUIT_SHIELD_PINOUT

  // bit 6/7 (PH3 & 4)
  // first two bits 0 & 1 (PH5 & 6)
 d = (PINH & 0x60) >> 5;
 d |= (PINH & 0x18) << 3;

  // bits 2 & 3 & 5 (PB4 & PB5, PB7)
 d |= (PINB & 0xB0) >> 2;

  // bit 4  (PG5)
  if (PING & _BV(5))
    d |= _BV(4);

#else
 d = MEGA_DATAPIN;
#endif

#else

  #error "No pins defined!"

#endif

 return d;
}

uint16_t tft_readData() {
 uint16_t d = 0;

  *portOutputRegister(csport) &= ~cspin;
  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |= cdpin;
  //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  *portOutputRegister(wrport) |= wrpin;
  //digitalWrite(_wr, HIGH);

  setReadDir();

  *portOutputRegister(rdport) &= ~rdpin;
  //digitalWrite(_rd, LOW);

  delayMicroseconds(10);
  d = read8();
  d <<= 8;

  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  *portOutputRegister(rdport) &= ~rdpin;
  //digitalWrite(_rd, LOW);

  delayMicroseconds(10);
  d |= read8();

  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);

  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);

  return d;
}

uint16_t tft_readRegister(uint16_t addr) {
   writeCommand(addr);
   return readData();
}

*/



