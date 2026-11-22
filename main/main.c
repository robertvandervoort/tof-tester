// M5StickC Plus + up to four DFRobot SEN0628 8x8 matrix ToF sensors.
//
// Grove port: GND, 5V, GPIO32 SDA (yellow), GPIO33 SCL (white).
// Sensor addresses are 0x30..0x33. The switch must be in I2C mode, and the
// sensor power-cycled after the switch changes.
//
// The sensor's I2C pull-ups were removed so 5V cannot feed back into the
// ESP32. The grove bus uses the ESP32 internal pull-ups to 3.3V at 100kHz.
//
// USB serial is the console UART at 115200 8N1. Button A (GPIO37) cycles the
// sensor drawn on the screen. Button B (GPIO39) rescans. Serial 'n' / 's' do
// the same.

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"

#include "font8x8_basic.h"

static const char *TAG = "tof";

#define LCD_W 135
#define LCD_H 240
#define PIN_LCD_MOSI 15
#define PIN_LCD_SCLK 13
#define PIN_LCD_DC 23
#define PIN_LCD_CS 5
#define PIN_LCD_RST 18
#define PIN_SDA 32
#define PIN_SCL 33
#define PIN_PMIC_SDA 21
#define PIN_PMIC_SCL 22
#define PIN_BTN_A 37
#define PIN_BTN_B 39
#define AXP192_ADDR 0x34

#define I2C_HZ 100000
#define ADDR_FIRST 0x30
#define ADDR_LAST 0x33
#define SENSOR_SLOTS 4
#define MATRIX 8
#define MATRIX_POINTS (MATRIX * MATRIX)
#define DIST_INVALID 4000
#define FRAME_PERIOD_MS 200

#define CMD_SETMODE 1
#define CMD_ALLDATA 2
#define STATUS_OK 0x53
#define STATUS_FAIL 0x63

static const uint8_t PKT_SET_8X8[] = {0x55, 0x00, 0x05, CMD_SETMODE, 0x00, 0x00, 0x00, 0x08};
static const uint8_t PKT_GET_ALL[] = {0x55, 0x00, 0x01, CMD_ALLDATA};

#define COL_WHITE 0xFFFF
#define COL_BLACK 0x0000
#define COL_YELLOW 0xFFE0
#define COL_GREEN 0x07E0
#define COL_RED 0xF800
#define COL_CYAN 0x07FF
#define COL_GREY 0x4208
#define COL_ORANGE 0xFD20
#define COL_DARK 0x0841

typedef struct {
    uint8_t addr;
    bool used;
    bool ranging;
    uint16_t mm[MATRIX_POINTS];
    uint16_t min_mm;
    uint16_t max_mm;
    uint16_t center_mm;
    uint8_t fail_streak;
    uint32_t retry_ms;
    i2c_master_dev_handle_t dev;
} sensor_t;

static sensor_t g_sensors[SENSOR_SLOTS];
static int g_sensor_count;
static int g_view;
static uint8_t g_found[16];
static int g_found_count;
static uint8_t g_prev_found[16];
static int g_prev_found_count = -1;
static uint32_t g_last_scan_ms;
static uint32_t g_last_hb_ms;

static i2c_master_bus_handle_t g_grove;
static i2c_master_bus_handle_t g_pmic;
static i2c_master_dev_handle_t g_axp;
static esp_lcd_panel_handle_t g_panel;
static SemaphoreHandle_t g_lcd_done;
static uint16_t *g_fb;

static uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static bool IRAM_ATTR lcd_trans_done(esp_lcd_panel_io_handle_t io,
                                      esp_lcd_panel_io_event_data_t *edata, void *ctx) {
    (void)io;
    (void)edata;
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)ctx, &wake);
    return wake == pdTRUE;
}

static void fb_clear(uint16_t color) {
    for (int i = 0; i < LCD_W * LCD_H; ++i) {
        g_fb[i] = color;
    }
}

static void fill_rect(int x, int y, int w, int h, uint16_t color) {
    if (w <= 0 || h <= 0) {
        return;
    }
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = x + w;
    int y1 = y + h;
    if (x1 > LCD_W) {
        x1 = LCD_W;
    }
    if (y1 > LCD_H) {
        y1 = LCD_H;
    }
    for (int yy = y0; yy < y1; ++yy) {
        uint16_t *row = g_fb + yy * LCD_W;
        for (int xx = x0; xx < x1; ++xx) {
            row[xx] = color;
        }
    }
}

static void draw_char(int x, int y, char c, uint16_t color, int scale) {
    if ((unsigned char)c > 127) {
        c = '?';
    }
    const uint8_t *glyph = (const uint8_t *)font8x8_basic[(uint8_t)c];
    for (int row = 0; row < 8; ++row) {
        uint8_t bits = glyph[row];
        for (int col = 0; col < 8; ++col) {
            if (bits & (1u << col)) {
                fill_rect(x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

static void draw_str(int x, int y, const char *s, uint16_t color, int scale) {
    while (*s) {
        draw_char(x, y, *s, color, scale);
        x += 8 * scale;
        ++s;
    }
}

static void flush_fb(void) {
    esp_lcd_panel_draw_bitmap(g_panel, 0, 0, LCD_W, LCD_H, g_fb);
    xSemaphoreTake(g_lcd_done, pdMS_TO_TICKS(300));
}

static uint16_t heat(uint16_t mm) {
    if (mm < 20 || mm >= DIST_INVALID) {
        return COL_DARK;
    }
    float t = (mm - 20) / 1800.0f;
    if (t < 0) {
        t = 0;
    }
    if (t > 1) {
        t = 1;
    }
    uint8_t r;
    uint8_t g;
    uint8_t b;
    if (t < 0.5f) {
        r = 255;
        g = (uint8_t)(255 * (t / 0.5f));
        b = 0;
    } else {
        r = (uint8_t)(255 * (1.0f - (t - 0.5f) / 0.5f));
        g = (uint8_t)(180 * (1.0f - (t - 0.5f) / 0.5f));
        b = (uint8_t)(255 * ((t - 0.5f) / 0.5f));
    }
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static bool address_present(uint8_t addr) {
    for (int i = 0; i < g_found_count; ++i) {
        if (g_found[i] == addr) {
            return true;
        }
    }
    return false;
}

static int view_index(void);

static int find_sensor(uint8_t addr) {
    for (int i = 0; i < SENSOR_SLOTS; ++i) {
        if (g_sensors[i].used && g_sensors[i].addr == addr) {
            return i;
        }
    }
    return -1;
}

static void draw_screen(const char *status) {
    fb_clear(COL_BLACK);
    draw_str(4, 4, "SEN0628", COL_WHITE, 1);

    int x = 2;
    for (uint8_t addr = ADDR_FIRST; addr <= ADDR_LAST; ++addr) {
        bool on = address_present(addr);
        int slot = find_sensor(addr);
        bool viewing = on && slot >= 0 && slot == g_view && g_sensors[slot].used;
        uint16_t color = viewing ? COL_YELLOW : (on ? COL_GREEN : COL_GREY);
        char label[6];
        snprintf(label, sizeof(label), "%02X%s", addr, viewing ? "*" : (on ? "+" : "-"));
        draw_str(x, 18, label, color, 1);
        x += 32;
    }

    int extra_y = 32;
    for (int i = 0; i < g_found_count; ++i) {
        if (g_found[i] >= ADDR_FIRST && g_found[i] <= ADDR_LAST) {
            continue;
        }
        char extra[12];
        snprintf(extra, sizeof(extra), "also %02X", g_found[i]);
        draw_str(4, extra_y, extra, COL_ORANGE, 1);
        extra_y += 10;
        if (extra_y > 52) {
            break;
        }
    }

    sensor_t *shown = NULL;
    if (g_sensor_count > 0 && g_view >= 0 && g_view < SENSOR_SLOTS && g_sensors[g_view].used) {
        shown = &g_sensors[g_view];
    }

    if (!shown) {
        if (status) {
            draw_str(4, 48, status, COL_CYAN, 2);
        }
        draw_str(4, 80, "no I2C", COL_RED, 2);
        draw_str(4, 102, "switch = I2C", COL_WHITE, 1);
        draw_str(4, 114, "then power cycle", COL_WHITE, 1);
        draw_str(4, 220, "B rescan", COL_GREY, 1);
        flush_fb();
        return;
    }

    char num[16];
    if (status) {
        draw_str(4, 56, status, COL_CYAN, 2);
    } else if (!shown->ranging) {
        draw_str(4, 56, "no mode", COL_RED, 2);
    } else if (shown->center_mm == 0) {
        draw_str(4, 56, "-- mm", COL_WHITE, 2);
    } else {
        snprintf(num, sizeof(num), "%u", shown->center_mm);
        draw_str(4, 56, num, COL_WHITE, 2);
        draw_str(4 + (int)strlen(num) * 16, 64, "mm", COL_CYAN, 1);
    }

    if (shown->min_mm) {
        snprintf(num, sizeof(num), "mn %u", shown->min_mm);
    } else {
        snprintf(num, sizeof(num), "mn --");
    }
    draw_str(4, 80, num, COL_CYAN, 1);
    if (shown->max_mm) {
        snprintf(num, sizeof(num), "mx %u", shown->max_mm);
    } else {
        snprintf(num, sizeof(num), "mx --");
    }
    draw_str(70, 80, num, COL_CYAN, 1);

    snprintf(num, sizeof(num), "%d/%d %s", view_index(), g_sensor_count, shown->fail_streak ? "ERR" : "ok");
    draw_str(4, 94, num, COL_GREY, 1);

    const int cell = 14;
    const int gx = (LCD_W - MATRIX * cell) / 2;
    const int gy = 112;
    for (int y = 0; y < MATRIX; ++y) {
        for (int xcell = 0; xcell < MATRIX; ++xcell) {
            fill_rect(gx + xcell * cell, gy + y * cell, cell - 1, cell - 1,
                      heat(shown->mm[y * MATRIX + xcell]));
        }
    }
    draw_str(4, 228, "A next   B scan", COL_GREY, 1);
    flush_fb();
}

static esp_err_t axp_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(g_axp, buf, sizeof(buf), 50);
}

static esp_err_t axp_read(uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(g_axp, &reg, 1, val, 1, 50);
}

static void pmic_init(void) {
    i2c_master_bus_config_t bus = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = PIN_PMIC_SDA,
        .scl_io_num = PIN_PMIC_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &g_pmic));
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP192_ADDR,
        .scl_speed_hz = 400000,
        .scl_wait_us = 1000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_pmic, &dev, &g_axp));

    // LDO2 is the backlight, LDO3 powers the LCD. Both at 3.0V.
    // 0x12 bit pattern matches the M5StickC factory init: EXTEN, LDO2, LDO3, DCDC1.
    ESP_ERROR_CHECK(axp_write(0x28, 0xCC));
    uint8_t en = 0;
    ESP_ERROR_CHECK(axp_read(0x12, &en));
    ESP_ERROR_CHECK(axp_write(0x12, (en & 0xEF) | 0x4D));
    ESP_LOGI(TAG, "AXP192 LDO2/LDO3 on (reg12 was 0x%02X)", en);
}

static void lcd_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * LCD_H * (int)sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    g_lcd_done = xSemaphoreCreateBinary();
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 2,
        .on_color_trans_done = lcd_trans_done,
        .user_ctx = g_lcd_done,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(g_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(g_panel, 52, 40));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(g_panel, true));

    g_fb = heap_caps_malloc(LCD_W * LCD_H * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!g_fb) {
        ESP_LOGE(TAG, "framebuffer alloc failed");
        abort();
    }
}

static void buttons_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_BTN_A) | (1ULL << PIN_BTN_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static bool button_pressed(int pin, bool *prev) {
    bool down = gpio_get_level(pin) == 0;
    bool edge = down && !*prev;
    *prev = down;
    return edge;
}

static void grove_init(void) {
    i2c_master_bus_config_t bus = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &g_grove));
}

static bool read_exact(i2c_master_dev_handle_t dev, uint8_t *dst, size_t len) {
    while (len) {
        size_t chunk = len > 32 ? 32 : len;
        if (i2c_master_receive(dev, dst, chunk, 30) != ESP_OK) {
            return false;
        }
        dst += chunk;
        len -= chunk;
    }
    return true;
}

static void drain(i2c_master_dev_handle_t dev) {
    uint8_t junk[32];
    i2c_master_receive(dev, junk, sizeof(junk), 20);
}

static bool transact(i2c_master_dev_handle_t dev, const uint8_t *tx, size_t tx_len, uint8_t cmd,
                      uint8_t *payload, uint16_t payload_cap, uint16_t *payload_len, uint32_t timeout_ms) {
    if (i2c_master_transmit(dev, tx, tx_len, 50) != ESP_OK) {
        return false;
    }
    uint32_t start = millis();
    uint8_t status = 0xFF;
    while ((millis() - start) < timeout_ms) {
        uint8_t b = 0xFF;
        if (i2c_master_receive(dev, &b, 1, 20) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (b == 0xFF) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (b == STATUS_OK || b == STATUS_FAIL) {
            status = b;
            break;
        }
    }
    if (status != STATUS_OK && status != STATUS_FAIL) {
        drain(dev);
        return false;
    }
    uint8_t hdr[3];
    if (!read_exact(dev, hdr, sizeof(hdr)) || hdr[0] != cmd) {
        drain(dev);
        return false;
    }
    uint16_t length = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
    if (length > payload_cap) {
        drain(dev);
        return false;
    }
    if (length && !read_exact(dev, payload, length)) {
        drain(dev);
        return false;
    }
    if (payload_len) {
        *payload_len = length;
    }
    return status == STATUS_OK;
}

static bool set_mode_8x8(i2c_master_dev_handle_t dev) {
    uint8_t payload[8];
    uint16_t len = 0;
    return transact(dev, PKT_SET_8X8, sizeof(PKT_SET_8X8), CMD_SETMODE, payload, sizeof(payload), &len, 2500);
}

static bool read_matrix(sensor_t *sensor) {
    uint8_t raw[MATRIX_POINTS * 2];
    uint16_t len = 0;
    if (!transact(sensor->dev, PKT_GET_ALL, sizeof(PKT_GET_ALL), CMD_ALLDATA, raw, sizeof(raw), &len, 500)) {
        return false;
    }
    int points = (int)(len / 2);
    if (points <= 0) {
        return false;
    }
    if (points > MATRIX_POINTS) {
        points = MATRIX_POINTS;
    }
    for (int i = 0; i < MATRIX_POINTS; ++i) {
        sensor->mm[i] = DIST_INVALID;
    }
    for (int i = 0; i < points; ++i) {
        sensor->mm[i] = (uint16_t)raw[i * 2] | ((uint16_t)raw[i * 2 + 1] << 8);
    }

    uint16_t min_v = 0xFFFF;
    uint16_t max_v = 0;
    int valid = 0;
    for (int i = 0; i < points; ++i) {
        uint16_t v = sensor->mm[i];
        if (v < 20 || v >= DIST_INVALID) {
            continue;
        }
        if (v < min_v) {
            min_v = v;
        }
        if (v > max_v) {
            max_v = v;
        }
        ++valid;
    }
    sensor->min_mm = valid ? min_v : 0;
    sensor->max_mm = valid ? max_v : 0;

    const int mid[4] = {3 * MATRIX + 3, 3 * MATRIX + 4, 4 * MATRIX + 3, 4 * MATRIX + 4};
    uint32_t sum = 0;
    int n = 0;
    for (int i = 0; i < 4; ++i) {
        uint16_t v = sensor->mm[mid[i]];
        if (v >= 20 && v < DIST_INVALID) {
            sum += v;
            ++n;
        }
    }
    sensor->center_mm = n ? (uint16_t)(sum / (uint32_t)n) : 0;
    return true;
}

static void scan_bus(void) {
    g_found_count = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c_master_probe(g_grove, addr, 20) == ESP_OK) {
            if (g_found_count < (int)(sizeof(g_found) / sizeof(g_found[0]))) {
                g_found[g_found_count++] = addr;
            }
        }
    }
    g_last_scan_ms = millis();
}

static bool found_changed(void) {
    if (g_found_count != g_prev_found_count) {
        return true;
    }
    for (int i = 0; i < g_found_count; ++i) {
        if (g_found[i] != g_prev_found[i]) {
            return true;
        }
    }
    return false;
}

static void print_scan(void) {
    if (!found_changed() && g_prev_found_count >= 0) {
        return;
    }
    g_prev_found_count = g_found_count;
    for (int i = 0; i < g_found_count; ++i) {
        g_prev_found[i] = g_found[i];
    }
    printf("SCAN n=%d", g_found_count);
    if (g_found_count == 0) {
        printf(" none\n");
    } else {
        for (int i = 0; i < g_found_count; ++i) {
            printf(" 0x%02X", g_found[i]);
        }
        printf("\n");
    }
    fflush(stdout);
}

static void recount(void) {
    g_sensor_count = 0;
    int first = -1;
    for (int i = 0; i < SENSOR_SLOTS; ++i) {
        if (!g_sensors[i].used) {
            continue;
        }
        if (first < 0) {
            first = i;
        }
        ++g_sensor_count;
    }
    if (g_sensor_count == 0) {
        g_view = 0;
        return;
    }
    if (!g_sensors[g_view].used) {
        g_view = first;
    }
}

static int view_index(void) {
    int n = 0;
    for (int i = 0; i < SENSOR_SLOTS; ++i) {
        if (!g_sensors[i].used) {
            continue;
        }
        if (i == g_view) {
            return n + 1;
        }
        ++n;
    }
    return 0;
}

static void arm_sensor(sensor_t *sensor) {
    char line[20];
    snprintf(line, sizeof(line), "cfg %02X", sensor->addr);
    draw_screen(line);
    printf("MODE 0x%02X ", sensor->addr);
    fflush(stdout);
    // setRangingMode blocks in the DFRobot library until the sensor leaves
    // 0xFF, which can take a couple of seconds on a cold start.
    if (!set_mode_8x8(sensor->dev)) {
        sensor->ranging = false;
        printf("FAIL\n");
        fflush(stdout);
        return;
    }
    printf("ok, arming\n");
    fflush(stdout);
    // After the ack the sensor restarts its ranging loop. The DFRobot example
    // waits 5s before the first read.
    for (int sec = 5; sec > 0; --sec) {
        snprintf(line, sizeof(line), "arming %ds", sec);
        draw_screen(line);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    sensor->ranging = true;
    sensor->fail_streak = 0;
    sensor->retry_ms = 0;
}

static void adopt_new_sensors(void) {
    for (uint8_t addr = ADDR_FIRST; addr <= ADDR_LAST; ++addr) {
        if (!address_present(addr) || find_sensor(addr) >= 0) {
            continue;
        }
        int slot = -1;
        for (int i = 0; i < SENSOR_SLOTS; ++i) {
            if (!g_sensors[i].used) {
                slot = i;
                break;
            }
        }
        if (slot < 0) {
            break;
        }
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = I2C_HZ,
            .scl_wait_us = 20000,
        };
        i2c_master_dev_handle_t dev = NULL;
        if (i2c_master_bus_add_device(g_grove, &cfg, &dev) != ESP_OK) {
            ESP_LOGW(TAG, "add 0x%02X failed", addr);
            continue;
        }
        sensor_t *sensor = &g_sensors[slot];
        memset(sensor, 0, sizeof(*sensor));
        sensor->used = true;
        sensor->addr = addr;
        sensor->dev = dev;
        for (int i = 0; i < MATRIX_POINTS; ++i) {
            sensor->mm[i] = DIST_INVALID;
        }
        recount();
        if (g_sensor_count == 1) {
            g_view = slot;
        }
        arm_sensor(sensor);
    }
}

static void drop_sensor(sensor_t *sensor) {
    if (sensor->dev) {
        i2c_master_bus_rm_device(sensor->dev);
        sensor->dev = NULL;
    }
    memset(sensor, 0, sizeof(*sensor));
}

static void forget_missing_sensors(void) {
    for (int i = 0; i < SENSOR_SLOTS; ++i) {
        if (g_sensors[i].used && !address_present(g_sensors[i].addr)) {
            printf("DROP 0x%02X\n", g_sensors[i].addr);
            drop_sensor(&g_sensors[i]);
        }
    }
    recount();
}

static void rescan(bool drop_missing) {
    scan_bus();
    print_scan();
    if (drop_missing) {
        forget_missing_sensors();
    }
    adopt_new_sensors();
}

static void print_matrix(const sensor_t *sensor, bool fresh) {
    if (!fresh) {
        printf("MAT 0x%02X %s\n", sensor->addr, sensor->ranging ? "wait" : "mode-fail");
        fflush(stdout);
        return;
    }
    printf("MAT 0x%02X ok min=%u max=%u ctr=%u\n", sensor->addr, sensor->min_mm, sensor->max_mm, sensor->center_mm);
    for (int y = 0; y < MATRIX; ++y) {
        printf("r%d", y);
        for (int x = 0; x < MATRIX; ++x) {
            printf(" %4u", sensor->mm[y * MATRIX + x]);
        }
        printf("\n");
    }
    fflush(stdout);
}

static void next_view(void) {
    if (g_sensor_count <= 0) {
        return;
    }
    for (int step = 1; step <= SENSOR_SLOTS; ++step) {
        int i = (g_view + step) % SENSOR_SLOTS;
        if (g_sensors[i].used) {
            g_view = i;
            return;
        }
    }
}

static void service_serial(void) {
    int c;
    while ((c = getchar()) != EOF) {
        if (c == 'n' || c == 'N') {
            next_view();
        } else if (c == 's' || c == 'S') {
            rescan(true);
        }
    }
}

static void stdin_nonblock(void) {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
}

void app_main(void) {
    esp_log_level_set("i2c.master", ESP_LOG_WARN);
    esp_log_level_set("lcd_panel.io.spi", ESP_LOG_WARN);
    esp_log_level_set("lcd_panel.st7789", ESP_LOG_WARN);

    printf("\nBOOT tof-tester SEN0628 (esp-idf)\n");
    printf("BUS SDA=%d SCL=%d %dkHz pullup=internal-3v3\n", PIN_SDA, PIN_SCL, I2C_HZ / 1000);
    printf("Expect SEN0628 at 0x30-0x33, switch in I2C mode\n");
    fflush(stdout);

    pmic_init();
    lcd_init();
    buttons_init();
    grove_init();
    stdin_nonblock();

    draw_screen("scanning");
    vTaskDelay(pdMS_TO_TICKS(400));
    rescan(true);
    draw_screen(NULL);

    bool prev_a = false;
    bool prev_b = false;
    while (true) {
        service_serial();
        if (button_pressed(PIN_BTN_A, &prev_a)) {
            next_view();
        }
        if (button_pressed(PIN_BTN_B, &prev_b)) {
            rescan(true);
        }
        if ((millis() - g_last_scan_ms) > 2000) {
            rescan(false);
        }

        uint32_t t0 = millis();
        if (g_sensor_count == 0) {
            if ((t0 - g_last_hb_ms) > 1000) {
                printf("HB t=%lu found=%d\n", (unsigned long)t0, g_found_count);
                fflush(stdout);
                g_last_hb_ms = t0;
            }
        }

        for (int i = 0; i < SENSOR_SLOTS; ++i) {
            sensor_t *sensor = &g_sensors[i];
            if (!sensor->used) {
                continue;
            }
            bool fresh = false;
            if (!sensor->ranging) {
                if ((millis() - sensor->retry_ms) > 1000) {
                    sensor->retry_ms = millis();
                    if (set_mode_8x8(sensor->dev)) {
                        sensor->ranging = true;
                        sensor->fail_streak = 0;
                        sensor->retry_ms = millis();
                    }
                }
            } else if ((millis() - sensor->retry_ms) < 5000) {
                // Still inside the post-mode settle window.
            } else if (read_matrix(sensor)) {
                sensor->fail_streak = 0;
                fresh = true;
            } else if (sensor->fail_streak < 250) {
                ++sensor->fail_streak;
                if (sensor->fail_streak == 8) {
                    sensor->ranging = false;
                }
            }
            print_matrix(sensor, fresh);
        }
        draw_screen(NULL);

        uint32_t elapsed = millis() - t0;
        if (elapsed < FRAME_PERIOD_MS) {
            vTaskDelay(pdMS_TO_TICKS(FRAME_PERIOD_MS - elapsed));
        }
    }
}
