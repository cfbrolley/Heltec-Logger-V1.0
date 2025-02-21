/****************************************/
/* TFTscreen library has been butchered from the HT_ST7735 library, 
 * which was butchered from the Adafruit ST7735 library,
 * which was probably butchered from something else.
 * will eventually strip out the unnecessary stuff and only leave in what's necessary for this logger
 * will also add fonts that don't look like they're from a Windows 3.1 screensaver
/****************************************/
#ifndef TFTscreen_H
#define TFTscreen_H
#include <SPI.h>
#include <stdbool.h>
#include "TFTfonts.h"
#include "Arduino.h"

#define TFT_MADCTL_MY  0x80
#define TFT_MADCTL_MX  0x40
#define TFT_MADCTL_MV  0x20
#define TFT_MADCTL_ML  0x10
#define TFT_MADCTL_RGB 0x00
#define TFT_MADCTL_BGR 0x08
#define TFT_MADCTL_MH  0x04

/*** Pin definitions for Heltec Wireless Tracker v1.1 ***/
#define TFT_CS_Pin        38
#define TFT_REST_Pin      39
#define TFT_DC_Pin        40
#define TFT_SCLK_Pin      41
#define TFT_MOSI_Pin      42
#define TFT_LED_K_Pin     21
#define TFT_VTFT_CTRL_Pin  3


// mini 160x80, rotate left

#define TFT_IS_160X80 1
#define TFT_XSTART 1
#define TFT_YSTART 26
#define TFTWIDTH  160
#define TFTHEIGHT 80
#define TFTROTATION (TFT_MADCTL_MY | TFT_MADCTL_MV | TFT_MADCTL_BGR)



/****************************/

#define TFT_NOP     0x00
#define TFT_SWRESET 0x01
#define TFT_RDDID   0x04
#define TFT_RDDST   0x09

#define TFT_SLPIN   0x10
#define TFT_SLPOUT  0x11
#define TFT_PTLON   0x12
#define TFT_NORON   0x13

#define TFT_INVOFF  0x20
#define TFT_INVON   0x21
#define TFT_GAMSET  0x26
#define TFT_DISPOFF 0x28
#define TFT_DISPON  0x29
#define TFT_CASET   0x2A
#define TFT_RASET   0x2B
#define TFT_RAMWR   0x2C
#define TFT_RAMRD   0x2E

#define TFT_PTLAR   0x30
#define TFT_COLMOD  0x3A
#define TFT_MADCTL  0x36

#define TFT_FRMCTR1 0xB1
#define TFT_FRMCTR2 0xB2
#define TFT_FRMCTR3 0xB3
#define TFT_INVCTR  0xB4
#define TFT_DISSET5 0xB6

#define TFT_PWCTR1  0xC0
#define TFT_PWCTR2  0xC1
#define TFT_PWCTR3  0xC2
#define TFT_PWCTR4  0xC3
#define TFT_PWCTR5  0xC4
#define TFT_VMCTR1  0xC5

#define TFT_RDID1   0xDA
#define TFT_RDID2   0xDB
#define TFT_RDID3   0xDC
#define TFT_RDID4   0xDD

#define TFT_PWCTR6  0xFC

#define TFT_GMCTRP1 0xE0
#define TFT_GMCTRN1 0xE1

// Color definitions
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

typedef enum {
	GAMMA_10 = 0x01,
	GAMMA_25 = 0x02,
	GAMMA_22 = 0x04,
	GAMMA_18 = 0x08
} GammaDef;


// call before initializing any SPI devices

class TFTscreen
{
private:
	/* data */
	void TFT_select(void);
	void TFT_unselect(void);
	void TFT_reset(void);
	void TFT_write_cmd(uint8_t cmd);
	void TFT_write_data(uint8_t* buff, size_t buff_size);
	void TFT_execute_cmd_list(const uint8_t *addr);
	void TFT_set_address_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	int8_t 	  _cs_pin;
	int8_t    _rest_pin;     
	int8_t    _dc_pin;    
	int8_t    _sclk_pin;   
	int8_t    _mosi_pin;  
	int8_t    _led_k_pin; 
	int8_t    _vtft_ctrl_pin;
	uint16_t _width;
	uint16_t _height;
	uint16_t _x_start;
	uint16_t _y_start;
public:
	TFTscreen(	int8_t 	  cs_pin=TFT_CS_Pin,
				int8_t    rest_pin=TFT_REST_Pin,     
				int8_t    dc_pin=TFT_DC_Pin,    
				int8_t    sclk_pin=TFT_SCLK_Pin,   
				int8_t    mosi_pin=TFT_MOSI_Pin,  
				int8_t    led_k_pin=TFT_LED_K_Pin,
				int8_t    vtft_ctrl_pin=TFT_VTFT_CTRL_Pin );
	~TFTscreen();
	void TFT_init(void);
	void draw_pixel(uint16_t x, uint16_t y, uint16_t color);
	void TFT_write_char(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
	void write_str(uint16_t x, uint16_t y, String str_data, FontDef font=Font_Medium, uint16_t color=BLUE, uint16_t bgcolor=BLACK);
	void write_str(uint16_t x, uint16_t y, const char *str, FontDef font=Font_Medium, uint16_t color=BLUE, uint16_t bgcolor=BLACK);
	void TFT_fill_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
	void fill_screen(uint16_t color);
	void TFT_draw_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
	void TFT_invert_colors(bool invert);
	void TFT_set_gamma(GammaDef gamma);
	void write_cmd(uint8_t test);
};


#endif // __TFT_H__
