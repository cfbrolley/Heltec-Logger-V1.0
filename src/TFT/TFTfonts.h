/* vim: set ai et ts=4 sw=4: */
#ifndef TFTFONTS_H
#define TFTFONTS_H
#include <stdint.h>

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;

extern FontDef Font_Small;
extern FontDef Font_Medium;
extern FontDef Font_Large;

#endif
