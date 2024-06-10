#ifndef _NV3401A_LCD_H
#define _NV3401A_LCD_H
#include <stdio.h>

class nv3401a_lcd
{
public:
    nv3401a_lcd(int8_t qspi_cs, int8_t qspi_clk, int8_t qspi_0,
                  int8_t qspi_1, int8_t qspi_2, int8_t qspi_3, int8_t lcd_rst);

    void begin();
    void lcd_draw_bitmap(uint16_t x_start, uint16_t y_start,
                         uint16_t x_end, uint16_t y_end, uint16_t *color_data);
    void draw16bitbergbbitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *color_data);
    void fillScreen(uint16_t color);
    uint16_t width();
    uint16_t height();

private:
    int8_t _qspi_cs, _qspi_clk, _qspi_0, _qspi_1, _qspi_2, _qspi_3, _lcd_rst;
};
#endif