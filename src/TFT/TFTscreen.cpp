/* vim: set ai et ts=4 sw=4: */

#include "TFTscreen.h"
#include "malloc.h"
#include "string.h"

SPIClass TFT_spi(HSPI);

// based on Adafruit ST7735 library for Arduino
#define DELAY 0x80

	const uint8_t  init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    TFT_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //      150 ms delay
    TFT_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    TFT_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    TFT_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    TFT_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    TFT_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    TFT_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    TFT_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    TFT_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    TFT_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    TFT_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    TFT_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    TFT_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    TFT_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
    TFTROTATION,        //     row addr/col addr, bottom to top refresh
    TFT_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 };                 //     16-bit color


#ifdef TFT_IS_160X80
  const uint8_t init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    TFT_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x4F,             //     XEND = 79
    TFT_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F ,            //     XEND = 159
    TFT_INVON, 0 };        //  3: Invert colors
#endif

  const uint8_t init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    TFT_GMCTRP1, 16      , //  1: Gamma Adjustments (pos. polarity), 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    TFT_GMCTRN1, 16      , //  2: Gamma Adjustments (neg. polarity), 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    TFT_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    TFT_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100   //     100 ms delay
};

TFTscreen::TFTscreen(	int8_t cs_pin,int8_t rest_pin, int8_t  dc_pin,    
	                    int8_t sclk_pin,int8_t mosi_pin, int8_t led_k_pin,
                      int8_t    vtft_ctrl_pin )
{
	_cs_pin = cs_pin;
	_rest_pin = rest_pin;     
	_dc_pin = dc_pin;    
	_sclk_pin = sclk_pin;   
	_mosi_pin =  mosi_pin;  
	_led_k_pin = led_k_pin; 
  _vtft_ctrl_pin = vtft_ctrl_pin;
  _width = TFTWIDTH;
  _height = TFTHEIGHT;
  _x_start = TFT_XSTART;
	_y_start = TFT_YSTART;
}

TFTscreen::~TFTscreen()
{

}

void TFTscreen::TFT_select(void) //these select and unselect functions are dumb. why is there a whole function call thatjust calls another function to set a pin high or low?
{
  digitalWrite(_cs_pin, LOW);
}

void TFTscreen::TFT_unselect(void)
{
  digitalWrite(_cs_pin, HIGH);
}

void TFTscreen::TFT_reset(void) 
{
  digitalWrite(_rest_pin, LOW);    
  delay(5);
  digitalWrite(_rest_pin, HIGH);    
}

void TFTscreen::TFT_write_cmd(uint8_t cmd)
{
    digitalWrite(_dc_pin, LOW);   
		TFT_spi.transfer(cmd);
}

void TFTscreen::TFT_write_data(uint8_t* buff, size_t buff_size) 
{
    digitalWrite(_dc_pin, HIGH); 
    TFT_spi.transfer(buff, buff_size);
}

void TFTscreen::TFT_execute_cmd_list(const uint8_t *addr)
{
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--) {
        uint8_t cmd = *addr++;
        TFT_write_cmd(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs) {
            TFT_write_data((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms) {
            ms = *addr++;
            if(ms == 255) ms = 500;
            delay(ms);
        }
    }
}

void TFTscreen::TFT_set_address_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) 
{
    // column address set
    TFT_write_cmd(TFT_CASET);
    uint8_t data[] = { 0x00, x0 + _x_start, 0x00, x1 + _x_start };
    TFT_write_data(data, sizeof(data));

    // row address set
    TFT_write_cmd(TFT_RASET);
    data[1] = y0 + _y_start;
    data[3] = y1 + _y_start;
    TFT_write_data(data, sizeof(data));

    // write to RAM
    TFT_write_cmd(TFT_RAMWR);
}

void TFTscreen::TFT_init(void) 
{
    if((_dc_pin <0 ) || (_cs_pin <0 ) || (_rest_pin <0 ) || (_led_k_pin <0 ))
    {
      printf("Pin error!\r\n");
      return;
    }
    
    if(_vtft_ctrl_pin >= 0)
    {
      pinMode(_vtft_ctrl_pin, OUTPUT);
      digitalWrite(_vtft_ctrl_pin, HIGH); 
    }

	  pinMode(_dc_pin, OUTPUT);  
    pinMode(_cs_pin, OUTPUT);
    pinMode(_rest_pin, OUTPUT);

    pinMode(_led_k_pin, OUTPUT);
    digitalWrite(_led_k_pin, HIGH);

    TFT_spi.begin(_sclk_pin,-1,_mosi_pin);
    TFT_select();
    TFT_reset();
    TFT_execute_cmd_list(init_cmds1);
    TFT_execute_cmd_list(init_cmds2);
    TFT_execute_cmd_list(init_cmds3);
    TFT_unselect();
}

void TFTscreen::draw_pixel(uint16_t x, uint16_t y, uint16_t color) 
{
    if((x >= _width) || (y >= _height))
    {
        return;
    }

    TFT_select();

    TFT_set_address_window(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    TFT_write_data(data, sizeof(data));

    TFT_unselect();
}

void TFTscreen::TFT_write_char(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) 
{
    uint32_t i, b, j;

    TFT_set_address_window(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                TFT_write_data(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                TFT_write_data(data, sizeof(data));
            }
        }
    }
}
void  TFTscreen::write_str(uint16_t x, uint16_t y, String str_data, FontDef font, uint16_t color, uint16_t bgcolor) 
{
    const char *str=str_data.c_str();
    write_str( x, y, str,  font,  color,  bgcolor);
}

void  TFTscreen::write_str(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor) 
{
    TFT_select();
    while(*str) {
        if(x + font.width >= _width) {
            x = 0;
            y += font.height;
            if(y + font.height >= _height) {
                break;
            }
            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }
        TFT_write_char(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    TFT_unselect();
}

void TFTscreen::TFT_fill_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= _width) || (y >= _height)) return;
    if((x + w - 1) >= _width) w = _width - x;
    if((y + h - 1) >= _height) h = _height - y;

    TFT_select();
    TFT_set_address_window(x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    digitalWrite(TFT_DC_Pin, HIGH); 
    for(y = h; y > 0; y--) 
    {
        for(x = w; x > 0; x--) 
        {
            TFT_spi.transfer(data, sizeof(data));
        }
    }

    TFT_unselect();
}


void TFTscreen::fill_screen(uint16_t color) 
{
    TFT_fill_rectangle(0, 0, _width, _height, color);
}


void TFTscreen::TFT_draw_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) 
{
    if((x >= _width) || (y >= _height)) return;
    if((x + w - 1) >= _width) return;
    if((y + h - 1) >= _height) return;

    TFT_select();
    TFT_set_address_window(x, y, x+w-1, y+h-1);
    TFT_write_data((uint8_t*)data, sizeof(uint16_t)*w*h);
    TFT_unselect();
}

void TFTscreen::TFT_invert_colors(bool invert) 
{
    TFT_select();
    TFT_write_cmd(invert ? TFT_INVON : TFT_INVOFF);
    TFT_unselect();
}

void TFTscreen::TFT_set_gamma(GammaDef gamma)
{
  uint8_t data[1];
  data[0] = (uint8_t)gamma;
	TFT_select();
	TFT_write_cmd(TFT_GAMSET);
	TFT_write_data(data, sizeof(data));
	TFT_unselect();
}

void TFTscreen::write_cmd(uint8_t cmd)
{
 TFT_write_cmd(cmd);
}