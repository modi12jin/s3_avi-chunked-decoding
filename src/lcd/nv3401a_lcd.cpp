#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_nv3401a.h"
#include "nv3401a_lcd.h"
#include "Arduino.h"

#define LCD_HOST SPI2_HOST
#define LCD_BIT_PER_PIXEL (16)

#define LCD_H_RES 480
#define LCD_V_RES 272

static const char *TAG = "example";
esp_lcd_panel_handle_t panel_handle = NULL;

nv3401a_lcd::nv3401a_lcd(int8_t qspi_cs, int8_t qspi_clk, int8_t qspi_0,
                         int8_t qspi_1, int8_t qspi_2, int8_t qspi_3, int8_t lcd_rst)
{
    _qspi_cs = qspi_cs;
    _qspi_clk = qspi_clk;
    _qspi_0 = qspi_0;
    _qspi_1 = qspi_1;
    _qspi_2 = qspi_2;
    _qspi_3 = qspi_3;
    _lcd_rst = lcd_rst;
}

void nv3401a_lcd::begin()
{
    const spi_bus_config_t buscfg = NV3401A_PANEL_BUS_QSPI_CONFIG(_qspi_clk,
                                                                  _qspi_0,
                                                                  _qspi_1,
                                                                  _qspi_2,
                                                                  _qspi_3,
                                                                  LCD_H_RES * LCD_V_RES * LCD_BIT_PER_PIXEL / 8);

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = NV3401A_PANEL_IO_QSPI_CONFIG(_qspi_cs, NULL, NULL);

    nv3401a_vendor_config_t vendor_config = {
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    // 将 LCD 连接到 SPI 总线
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = _lcd_rst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };

    esp_lcd_new_panel_nv3401a(io_handle, &panel_config, &panel_handle);

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    // 在打开屏幕或背光之前，用户可以将预定义的图案刷新到屏幕上
    esp_lcd_panel_disp_on_off(panel_handle, true);
}

void nv3401a_lcd::lcd_draw_bitmap(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *color_data)
{
    esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, color_data);
}

void nv3401a_lcd::draw16bitbergbbitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *color_data)
{
    uint16_t x_start = x;
    uint16_t y_start = y;
    uint16_t x_end = w + x;
    uint16_t y_end = h + y;

    esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, color_data);
}

void nv3401a_lcd::fillScreen(uint16_t color)
{
    uint16_t *color_data = (uint16_t *)heap_caps_malloc(480 * 272 * 2, MALLOC_CAP_INTERNAL);
    memset(color_data, color, 480 * 272 * 2);
    draw16bitbergbbitmap(0, 0, 480, 272, color_data);
    free(color_data);
}

uint16_t nv3401a_lcd::width()
{
    return LCD_H_RES;
}

uint16_t nv3401a_lcd::height()
{
    return LCD_V_RES;
}