/*
 * tft.h
 *
 *  Created on: Sep 24, 2019
 *      Author: Sigit
 */

#ifndef TFT_H_
#define TFT_H_

#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#define DATAPORT1 PORTD
#define DATAPIN1  PIND
#define DATADDR1  DDRD
#define DATAPORT2 PORTB
#define DATAPIN2  PINB
#define DATADDR2  DDRB
#define DATA1_MASK 0xFC  // top 6 bits
#define DATA2_MASK 0x03  // bottom 2 bits

// Port Pins
#define TFT_PORT				GPIOB

#define TFT_RST_PIN				GPIO_Pin_12
#define TFT_CS_PIN				GPIO_Pin_13
#define TFT_CD_PIN				GPIO_Pin_14
#define TFT_WR_PIN			    GPIO_Pin_15
#define TFT_RD_PIN			    GPIO_Pin_3

#define SET_RST					GPIO_SetBits(TFT_PORT, TFT_RST_PIN)
#define RESET_RST				GPIO_ResetBits(TFT_PORT, TFT_RST_PIN)

#define SET_CS					GPIO_SetBits(TFT_PORT, TFT_CS_PIN)
#define RESET_CS				GPIO_ResetBits(TFT_PORT, TFT_CS_PIN)

#define SET_CD					GPIO_SetBits(TFT_PORT, TFT_CD_PIN)
#define RESET_CD				GPIO_ResetBits(TFT_PORT, TFT_CD_PIN)

#define SET_WR				    GPIO_SetBits(TFT_PORT, TFT_WR_PIN)
#define RESET_WR				GPIO_ResetBits(TFT_PORT, TFT_WR_PIN)

#define SET_RD				    GPIO_SetBits(TFT_PORT, TFT_RD_PIN)
#define RESET_RD				GPIO_ResetBits(TFT_PORT, TFT_RD_PIN)

//#define cs_pin_init()					LCD_DDR |= _BV(LCD_CS_PIN)
#define cs_pin_low()					RESET_CS
#define cs_pin_high()					SET_CS

//#define cd_pin_init()					LCD_DDR |= _BV(LCD_CD_PIN)
#define cd_pin_low()					RESET_CD
#define cd_pin_high()					SET_CD

//#define wr_pin_init()					LCD_DDR |= _BV(LCD_WR_PIN)
#define wr_pin_low()					RESET_WR
#define wr_pin_high()					SET_WR

//#define rd_pin_init()					LCD_DDR |= _BV(LCD_RD_PIN)
#define rd_pin_low()					RESET_RD
#define rd_pin_high()					SET_RD

//#define reset_pin_init()				LCD_DDR |= _BV(LCD_RESET_PIN)
#define reset_pin_low()					RESET_RST
#define reset_pin_high()				SET_RST

#define TFTLCD_START_OSC			0x00
#define TFTLCD_DRIV_OUT_CTRL		0x01
#define TFTLCD_DRIV_WAV_CTRL		0x02
#define TFTLCD_ENTRY_MOD			0x03
#define TFTLCD_RESIZE_CTRL			0x04
#define TFTLCD_DISP_CTRL1			0x07
#define TFTLCD_DISP_CTRL2			0x08
#define TFTLCD_DISP_CTRL3			0x09
#define TFTLCD_DISP_CTRL4			0x0A
#define TFTLCD_RGB_DISP_IF_CTRL1	0x0C
#define TFTLCD_FRM_MARKER_POS		0x0D
#define TFTLCD_RGB_DISP_IF_CTRL2	0x0F
#define TFTLCD_POW_CTRL1			0x10
#define TFTLCD_POW_CTRL2			0x11
#define TFTLCD_POW_CTRL3			0x12
#define TFTLCD_POW_CTRL4			0x13
#define TFTLCD_GRAM_HOR_AD			0x20
#define TFTLCD_GRAM_VER_AD			0x21
#define TFTLCD_RW_GRAM				0x22
#define TFTLCD_POW_CTRL7			0x29
#define TFTLCD_FRM_RATE_COL_CTRL	0x2B
#define TFTLCD_GAMMA_CTRL1			0x30
#define TFTLCD_GAMMA_CTRL2			0x31
#define TFTLCD_GAMMA_CTRL3			0x32
#define TFTLCD_GAMMA_CTRL4			0x35
#define TFTLCD_GAMMA_CTRL5			0x36
#define TFTLCD_GAMMA_CTRL6			0x37
#define TFTLCD_GAMMA_CTRL7			0x38
#define TFTLCD_GAMMA_CTRL8			0x39
#define TFTLCD_GAMMA_CTRL9			0x3C
#define TFTLCD_GAMMA_CTRL10			0x3D
#define TFTLCD_HOR_START_AD			0x50
#define TFTLCD_HOR_END_AD			0x51
#define TFTLCD_VER_START_AD			0x52
#define TFTLCD_VER_END_AD			0x53
#define TFTLCD_GATE_SCAN_CTRL1		0x60
#define TFTLCD_GATE_SCAN_CTRL2		0x61
#define TFTLCD_GATE_SCAN_CTRL3		0x6A
#define TFTLCD_PART_IMG1_DISP_POS	0x80
#define TFTLCD_PART_IMG1_START_AD	0x81
#define TFTLCD_PART_IMG1_END_AD		0x82
#define TFTLCD_PART_IMG2_DISP_POS	0x83
#define TFTLCD_PART_IMG2_START_AD	0x84
#define TFTLCD_PART_IMG2_END_AD		0x85
#define TFTLCD_PANEL_IF_CTRL1		0x90
#define TFTLCD_PANEL_IF_CTRL2		0x92
#define TFTLCD_PANEL_IF_CTRL3		0x93
#define TFTLCD_PANEL_IF_CTRL4		0x95
#define TFTLCD_PANEL_IF_CTRL5		0x97
#define TFTLCD_PANEL_IF_CTRL6		0x98

#define swap(a, b) { int16_t t = a; a = b; b = t; }


static const uint16_t TFTWIDTH = 240;
static const uint16_t TFTHEIGHT = 320;
uint16_t _width, _height;
uint8_t rotation;
uint8_t textsize;
uint16_t cursor_x, cursor_y;
uint16_t textcolor;

void tft_begin();
void tft_reset(void);
void tft_initDisplay(void);
void tft_fillScreen(uint16_t color);
void tft_setCursor(uint16_t x, uint16_t y);
void tft_setTextSize(uint8_t s);
void tft_setTextColor(uint16_t c);
void tft_write(uint8_t);
void tft_print(const char *str);
void tft_println(const char *str);
void tft_setRotation(uint8_t x);
void tft_fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fillcolor);
void tft_drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void tft_draw565CutoutBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* data);


#endif /* TFT_H_ */
