#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "font8x8_basic.h"
#include "lcd.h"

#define LCD_HOST    SPI2_HOST
#define PIN_NUM_MOSI 21
#define PIN_NUM_CLK  17
#define PIN_NUM_CS   15
#define PIN_NUM_DC   33
#define PIN_NUM_RST  34
#define PIN_NUM_BL   16
#define LCD_WIDTH    128
#define LCD_HEIGHT   128

static spi_device_handle_t lcd_spi;

static void lcd_cmd(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void*)0
    };
    gpio_set_level(PIN_NUM_DC, 0); // Command mode
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(lcd_spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
}

static void lcd_data(const uint8_t *data, int len)
{
    if (len == 0) return;
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
        .user = (void*)1
    };
    gpio_set_level(PIN_NUM_DC, 1); // Data mode
    gpio_set_level(PIN_NUM_CS, 0);
    spi_device_transmit(lcd_spi, &t);
    gpio_set_level(PIN_NUM_CS, 1);
}

static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];
    lcd_cmd(0x2A); // column addr set
    data[0] = (x0 >> 8); data[1] = (x0 & 0xFF);
    data[2] = (x1 >> 8); data[3] = (x1 & 0xFF);
    lcd_data(data, 4);

    lcd_cmd(0x2B); // row addr set
    data[0] = (y0 >> 8); data[1] = (y0 & 0xFF);
    data[2] = (y1 >> 8); data[3] = (y1 & 0xFF);
    lcd_data(data, 4);

    lcd_cmd(0x2C); // write to RAM
}

void lcd_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2 + 8
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1, // 手動CS制御
        .queue_size = 7,
    };

    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(LCD_HOST, &devcfg, &lcd_spi);

    // GPIO初期化
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<PIN_NUM_DC) | (1ULL<<PIN_NUM_RST) | (1ULL<<PIN_NUM_CS) | (1ULL<<PIN_NUM_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_NUM_CS, 1);
    gpio_set_level(PIN_NUM_BL, 1); // バックライトON

    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 簡易なGC9107初期化コマンド（参考用）
    lcd_cmd(0xfe);  // Enter Extended Command Set
    lcd_cmd(0xef);  // Another special command
    lcd_cmd(0x36); uint8_t madctl = 0x00; lcd_data(&madctl, 1); // MADCTL
    lcd_cmd(0x3A); uint8_t colmod = 0x05; lcd_data(&colmod, 1); // 16-bit/pixel
    lcd_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120)); // Sleep out
    lcd_cmd(0x29); // Display ON
}

void draw_pixel(uint16_t x, uint16_t y, uint16_t color565)
{
    color565 ^= 0xffff;
    set_window(x, y, x, y);
    uint8_t data[] = { (color565 >> 8) & 0xFF, color565 & 0xFF };
    lcd_data(data, 2);
}

void draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (c < 0x20 || c > 0x7F) return;
    const uint8_t* glyph = (uint8_t*)font8x8_basic[(int)c];
    for (int row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        for (int col = 0; col < 8; col++) {
            uint16_t color = (bits & (1 << col)) ? fg : bg;
            for (int dx = 0; dx < scale; dx++) {
                for (int dy = 0; dy < scale; dy++) {
                    draw_pixel(x + col * scale + dx, y + row * scale + dy, color);
                }
            }
        }
    }
}

void draw_string(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint16_t char_w = 8 * scale;
    uint16_t char_h = 8 * scale;

    while (*str) {
        if (x + char_w > LCD_WIDTH) {
            x = 0;
            y += char_h;
        }
        if (y + char_h > LCD_HEIGHT) break;

        draw_char(x, y, *str++, fg, bg, scale);
        x += char_w;
    }
}

void draw_fill(uint16_t color565)
{
	color565 ^= 0xffff;

    set_window(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
    gpio_set_level(PIN_NUM_DC, 1);
    gpio_set_level(PIN_NUM_CS, 0);
    uint8_t data[] = { (color565 >> 8) & 0xFF, color565 & 0xFF };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = data,
    };
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        spi_device_transmit(lcd_spi, &t);
    }
    gpio_set_level(PIN_NUM_CS, 1);
}
