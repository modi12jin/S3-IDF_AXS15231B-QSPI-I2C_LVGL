/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_log.h"

#include "esp_lcd_axs15231b.h"

#define LCD_OPCODE_WRITE_CMD        (0x02ULL)
#define LCD_OPCODE_READ_CMD         (0x0BULL)
#define LCD_OPCODE_WRITE_COLOR      (0x32ULL)

static const char *TAG = "axs15231b";

static esp_err_t panel_axs15231b_del(esp_lcd_panel_t *panel);
static esp_err_t panel_axs15231b_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_axs15231b_init(esp_lcd_panel_t *panel);
static esp_err_t panel_axs15231b_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_axs15231b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_axs15231b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_axs15231b_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_axs15231b_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_axs15231b_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save surrent value of LCD_CMD_COLMOD register
    const axs15231b_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    struct {
        unsigned int use_qspi_interface: 1;
        unsigned int reset_level: 1;
    } flags;
} axs15231b_panel_t;

esp_err_t esp_lcd_new_panel_axs15231b(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    esp_err_t ret = ESP_OK;
    axs15231b_panel_t *axs15231b = NULL;
    axs15231b = calloc(1, sizeof(axs15231b_panel_t));
    ESP_GOTO_ON_FALSE(axs15231b, ESP_ERR_NO_MEM, err, TAG, "no mem for axs15231b panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order) {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        axs15231b->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        axs15231b->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color element order");
        break;
    }

    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        axs15231b->colmod_val = 0x55;
        axs15231b->fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        axs15231b->colmod_val = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        axs15231b->fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    axs15231b->io = io;
    axs15231b->reset_gpio_num = panel_dev_config->reset_gpio_num;
    axs15231b->flags.reset_level = panel_dev_config->flags.reset_active_high;
    axs15231b_vendor_config_t *vendor_config = (axs15231b_vendor_config_t *)panel_dev_config->vendor_config;
    if (vendor_config) {
        axs15231b->init_cmds = vendor_config->init_cmds;
        axs15231b->init_cmds_size = vendor_config->init_cmds_size;
        axs15231b->flags.use_qspi_interface = vendor_config->flags.use_qspi_interface;
    }
    axs15231b->base.del = panel_axs15231b_del;
    axs15231b->base.reset = panel_axs15231b_reset;
    axs15231b->base.init = panel_axs15231b_init;
    axs15231b->base.draw_bitmap = panel_axs15231b_draw_bitmap;
    axs15231b->base.invert_color = panel_axs15231b_invert_color;
    axs15231b->base.set_gap = panel_axs15231b_set_gap;
    axs15231b->base.mirror = panel_axs15231b_mirror;
    axs15231b->base.swap_xy = panel_axs15231b_swap_xy;
    axs15231b->base.disp_on_off = panel_axs15231b_disp_on_off;
    *ret_panel = &(axs15231b->base);
    ESP_LOGD(TAG, "new axs15231b panel @%p", axs15231b);

    return ESP_OK;

err:
    if (axs15231b) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(axs15231b);
    }
    return ret;
}

static esp_err_t tx_param(axs15231b_panel_t *axs15231b, esp_lcd_panel_io_handle_t io, int lcd_cmd, const void *param, size_t param_size)
{
    if (axs15231b->flags.use_qspi_interface) {
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
    }
    return esp_lcd_panel_io_tx_param(io, lcd_cmd, param, param_size);
}

static esp_err_t tx_color(axs15231b_panel_t *axs15231b, esp_lcd_panel_io_handle_t io, int lcd_cmd, const void *param, size_t param_size)
{
    if (axs15231b->flags.use_qspi_interface) {
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_COLOR << 24;
    }
    return esp_lcd_panel_io_tx_color(io, lcd_cmd, param, param_size);
}

static esp_err_t panel_axs15231b_del(esp_lcd_panel_t *panel)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);

    if (axs15231b->reset_gpio_num >= 0) {
        gpio_reset_pin(axs15231b->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del axs15231b panel @%p", axs15231b);
    free(axs15231b);
    return ESP_OK;
}

static esp_err_t panel_axs15231b_reset(esp_lcd_panel_t *panel)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);
    esp_lcd_panel_io_handle_t io = axs15231b->io;

    // Perform hardware reset
    if (axs15231b->reset_gpio_num >= 0) {
        gpio_set_level(axs15231b->reset_gpio_num, axs15231b->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
        gpio_set_level(axs15231b->reset_gpio_num, !axs15231b->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
        gpio_set_level(axs15231b->reset_gpio_num, axs15231b->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
    } else { // Perform software reset
        ESP_RETURN_ON_ERROR(tx_param(axs15231b, io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    return ESP_OK;
}

static const axs15231b_lcd_init_cmd_t vendor_specific_init_default[] = {
{0xBB, (uint8_t []){0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0xA5}, 8, 0},

{0xA0, (uint8_t []){0xC0,0x10,0x00,0x02,0x00,0x00,0x04,0x3F,0x20,0x05,0x3F,0x3F,0x00,0x00,0x00,0x00,0x00}, 17, 0},

{0xA2, (uint8_t []){0x30,0x3C,0x24,0x14,0xD0,0x20,0xFF,0xE0,0x40,0x19,0x80,0x80,0x80,0x20,0xf9,0x10,0x02,
0xff,0xff,0xF0,0x90,0x01,0x32,0xA0,0x91,0xE0,0x20,0x7F,0xFF,0x00,0x5A}, 31, 0},

{0xD0, (uint8_t []){0xE0,0x40,0x51,0x24,0x08,0x05,0x10,0x01,0x20,0x15,0x42,0xC2,0x22,0x22,0xAA,0x03,0x10,
0x12,0x60,0x14,0x1E,0x51,0x15,0x00,0x8A,0x20,0x00,0x03,0x3A,0x12}, 30, 0},

{0xA3, (uint8_t []){0xA0,0x06,0xAa,0x00,0x08,0x02,0x0A,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,
0x04,0x04,0x04,0x04,0x00,0x55,0x55}, 22, 0},

{0xC1, (uint8_t []){0x31,0x04,0x02,0x02,0x71,0x05,0x24,0x55,0x02,0x00,0x41,0x00,0x53,0xFF,0xFF,
0xFF,0x4F,0x52,0x00,0x4F,0x52,0x00,0x45,0x3B,0x0B,0x02,0x0d,0x00,0xFF,0x40}, 30, 0},

{0xC3, (uint8_t []){0x00,0x00,0x00,0x50,0x03,0x00,0x00,0x00,0x01,0x80,0x01}, 11, 0},

{0xC4, (uint8_t []){0x00,0x24,0x33,0x80,0x00,0xea,0x64,0x32,0xC8,0x64,0xC8,0x32,0x90,0x90,0x11,
0x06,0xDC,0xFA,0x00,0x00,0x80,0xFE,0x10,0x10,0x00,0x0A,0x0A,0x44,0x50}, 29, 0},

// TP
{0xC5, (uint8_t []){0x18,0x00,0x00,0x03,0xFE,0x3A,0x4A,0x20,0x30,0x10,0x88,0xDE,0x0D,0x08,0x0F,
0x0F,0x01,0x3A,0x4A,0x20,0x10,0x10,0x00}, 23, 0},

{0xC6, (uint8_t []){0x05,0x0A,0x05,0x0A,0x00,0xE0,0x2E,0x0B,0x12,0x22,0x12,0x22,
0x01,0x03,0x00,0x3F,0x6A,0x18,0xC8,0x22}, 20, 0},

{0xC7, (uint8_t []){0x50,0x32,0x28,0x00,0xa2,0x80,0x8f,0x00,0x80,0xff,0x07,
0x11,0x9c,0x67,0xff,0x24,0x0c,0x0d,0x0e,0x0f}, 20, 0},

{0xC9, (uint8_t []){0x33,0x44,0x44,0x01}, 4, 0},

{0xCF, (uint8_t []){0x2C,0x1E,0x88,0x58,0x13,0x18,0x56,0x18,0x1E,0x68,0x88,
0x00,0x65,0x09,0x22,0xC4,0x0C,0x77,0x22,0x44,0xAA,0x55,0x08,0x08,0x12,0xA0,0x08}, 27, 0},

{0xD5, (uint8_t []){0x40,0x8E,0x8D,0x01,0x35,0x04,0x92,0x74,0x04,0x92,0x74,
0x04,0x08,0x6A,0x04,0x46,0x03,0x03,0x03,0x03,0x82,0x01,0x03,0x00,0xE0,0x51,0xA1,0x00,0x00,0x00}, 30, 0},

{0xD6, (uint8_t []){0x10,0x32,0x54,0x76,0x98,0xBA,0xDC,0xFE,0x93,0x00,0x01,0x83,
0x07,0x07,0x00,0x07,0x07,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x84,0x00,0x20,0x01,0x00}, 30, 0},

{0xD7, (uint8_t []){0x03,0x01,0x0b,0x09,0x0f,0x0d,0x1E,0x1F,0x18,0x1d,0x1f,0x19,
0x40,0x8E,0x04,0x00,0x20,0xA0,0x1F}, 19, 0},

{0xD8, (uint8_t []){0x02,0x00,0x0a,0x08,0x0e,0x0c,0x1E,0x1F,0x18,0x1d,0x1f,0x19}, 12, 0},

{0xD9, (uint8_t []){0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}, 12, 0},

{0xDD, (uint8_t []){0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}, 12, 0},

{0xDF, (uint8_t []){0x44,0x73,0x4B,0x69,0x00,0x0A,0x02,0x90}, 8, 0},

{0xE0, (uint8_t []){0x3B,0x28,0x10,0x16,0x0c,0x06,0x11,0x28,0x5c,0x21,0x0D,0x35,0x13,0x2C,0x33,0x28,0x0D}, 17, 0},

{0xE1, (uint8_t []){0x37,0x28,0x10,0x16,0x0b,0x06,0x11,0x28,0x5C,0x21,0x0D,0x35,0x14,0x2C,0x33,0x28,0x0F}, 17, 0},

{0xE2, (uint8_t []){0x3B,0x07,0x12,0x18,0x0E,0x0D,0x17,0x35,0x44,0x32,0x0C,0x14,0x14,0x36,0x3A,0x2F,0x0D}, 17, 0},

{0xE3, (uint8_t []){0x37,0x07,0x12,0x18,0x0E,0x0D,0x17,0x35,0x44,0x32,0x0C,0x14,0x14,0x36,0x32,0x2F,0x0F}, 17, 0},

{0xE4, (uint8_t []){0x3B,0x07,0x12,0x18,0x0E,0x0D,0x17,0x39,0x44,0x2E,0x0C,0x14,0x14,0x36,0x3A,0x2F,0x0D}, 17, 0},

{0xE5, (uint8_t []){0x37,0x07,0x12,0x18,0x0E,0x0D,0x17,0x39,0x44,0x2E,0x0C,0x14,0x14,0x36,0x3A,0x2F,0x0F}, 17, 0},

{0xA4, (uint8_t []){0x85,0x85,0x95,0x82,0xAF,0xAA,0xAA,0x80,0x10,0x30,0x40,0x40,0x20,0xFF,0x60,0x30}, 16, 0},

{0xA4, (uint8_t []){0x85,0x85,0x95,0x85}, 4, 0},

{0xBB, (uint8_t []){0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 8, 0},

//{0x3A, (uint8_t []){0x01}, 1, 0},

{0x11, (uint8_t []){0x00}, 1, 120},

{0x29, (uint8_t []){0x00}, 1, 100},

{0x2C, (uint8_t []){0x00,0x00,0x00,0x00}, 4, 0},

//{0x22, (uint8_t[]){0x00}, 0, 200},//All Pixels off
};

static esp_err_t panel_axs15231b_init(esp_lcd_panel_t *panel)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);
    esp_lcd_panel_io_handle_t io = axs15231b->io;
    const axs15231b_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    bool is_user_set = true;
    bool is_cmd_overwritten = false;

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    ESP_RETURN_ON_ERROR(tx_param(axs15231b, io, LCD_CMD_SLPOUT, NULL, 0), TAG, "send command failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(tx_param(axs15231b, io, LCD_CMD_MADCTL, (uint8_t[]) {
        axs15231b->madctl_val,
    }, 1), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(tx_param(axs15231b, io, LCD_CMD_COLMOD, (uint8_t[]) {
        axs15231b->colmod_val,
    }, 1), TAG, "send command failed");

    // vendor specific initialization, it can be different between manufacturers
    // should consult the LCD supplier for initialization sequence code
    if (axs15231b->init_cmds) {
        init_cmds = axs15231b->init_cmds;
        init_cmds_size = axs15231b->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(axs15231b_lcd_init_cmd_t);
    }

    for (int i = 0; i < init_cmds_size; i++) {
        // Check if the command has been used or conflicts with the internal
        if (is_user_set && (init_cmds[i].data_bytes > 0)) {
            switch (init_cmds[i].cmd) {
            case LCD_CMD_MADCTL:
                is_cmd_overwritten = true;
                axs15231b->madctl_val = ((uint8_t *)init_cmds[i].data)[0];
                break;
            case LCD_CMD_COLMOD:
                is_cmd_overwritten = true;
                axs15231b->colmod_val = ((uint8_t *)init_cmds[i].data)[0];
                break;
            default:
                is_cmd_overwritten = false;
                break;
            }

            if (is_cmd_overwritten) {
                is_cmd_overwritten = false;
                ESP_LOGW(TAG, "The %02Xh command has been used and will be overwritten by external initialization sequence", init_cmds[i].cmd);
            }
        }

        // Send command
        ESP_RETURN_ON_ERROR(tx_param(axs15231b, io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
    }
    ESP_LOGD(TAG, "send init commands success");

    return ESP_OK;
}

static esp_err_t panel_axs15231b_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = axs15231b->io;

    x_start += axs15231b->x_gap;
    x_end += axs15231b->x_gap;
    y_start += axs15231b->y_gap;
    y_end += axs15231b->y_gap;

    // define an area of frame memory where MCU can access
    tx_param(axs15231b, io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4);
    // tx_param(axs15231b, io, LCD_CMD_RASET, (uint8_t[]) {
    //     (y_start >> 8) & 0xFF,
    //     y_start & 0xFF,
    //     ((y_end - 1) >> 8) & 0xFF,
    //     (y_end - 1) & 0xFF,
    // }, 4);
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * axs15231b->fb_bits_per_pixel / 8;

    if (y_start == 0) {
        tx_color(axs15231b, io, LCD_CMD_RAMWR, color_data, len);//2C
    } else {
        tx_color(axs15231b, io, LCD_CMD_RAMWRC, color_data, len);//2C
    }

    return ESP_OK;
}

static esp_err_t panel_axs15231b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);
    esp_lcd_panel_io_handle_t io = axs15231b->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(tx_param(axs15231b, io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_axs15231b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);
    esp_lcd_panel_io_handle_t io = axs15231b->io;
    esp_err_t ret = ESP_OK;

    if (mirror_x) {
        axs15231b->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        axs15231b->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        axs15231b->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        axs15231b->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(tx_param(axs15231b, io, LCD_CMD_MADCTL, (uint8_t[]) {
        axs15231b->madctl_val
    }, 1), TAG, "send command failed");
    return ret;
}

static esp_err_t panel_axs15231b_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);
    esp_lcd_panel_io_handle_t io = axs15231b->io;
    if (swap_axes) {
        axs15231b->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        axs15231b->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        axs15231b->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_axs15231b_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);
    axs15231b->x_gap = x_gap;
    axs15231b->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_axs15231b_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    axs15231b_panel_t *axs15231b = __containerof(panel, axs15231b_panel_t, base);
    esp_lcd_panel_io_handle_t io = axs15231b->io;
    int command = 0;

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(tx_param(axs15231b, io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}
