#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_touch_gt911.h"
#include "gt911_touch.h"

#define CONFIG_LCD_HRES 600
#define CONFIG_LCD_VRES 1024

static const char *TAG = "example";

esp_lcd_touch_handle_t tp;
esp_lcd_panel_io_handle_t tp_io_handle;

uint16_t touch_strength[1];
uint8_t touch_cnt = 0;

gt911_touch::gt911_touch(int8_t sda_pin, int8_t scl_pin, int8_t rst_pin, int8_t int_pin)
{
    _sda = sda_pin;
    _scl = scl_pin;
    _rst = rst_pin;
    _int = int_pin;
}

// The GT911 latches its I2C address from the INT pin at the instant RST is
// released: INT low selects 0x5D, INT high selects 0x14.
//
// esp_lcd_touch_gt911 configures INT as an input *before* it pulses RST, so at
// the moment of release the level is whatever the pin happens to float to. On
// a warm reboot the controller keeps the address it already latched and the
// driver works; on a cold power-on it can come up at 0x14 while the driver
// talks to 0x5D, which is why the first boot after power-up failed and the
// board only worked after the resulting panic reboot.
//
// Drive the sequence explicitly, then hand the driver an already-reset part.
void gt911_touch::selectI2cAddress()
{
    if (_rst < 0) return;

    const uint64_t pins = (_int >= 0)
        ? (BIT64((gpio_num_t)_rst) | BIT64((gpio_num_t)_int))
        : BIT64((gpio_num_t)_rst);

    const gpio_config_t out_cfg = {
        .pin_bit_mask = pins,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    // Hold both low, so the address bit is already settled before release.
    gpio_set_level((gpio_num_t)_rst, 0);
    if (_int >= 0) gpio_set_level((gpio_num_t)_int, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Release reset with INT still low -> address 0x5D.
    gpio_set_level((gpio_num_t)_rst, 1);

    // The datasheet wants INT held for at least 5 ms after release; give it
    // room, then let the pin go back to being an interrupt input.
    vTaskDelay(pdMS_TO_TICKS(60));

    if (_int >= 0) {
        const gpio_config_t in_cfg = {
            .pin_bit_mask = BIT64((gpio_num_t)_int),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&in_cfg));
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}

bool gt911_touch::begin()
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)_sda,
        .scl_io_num = (gpio_num_t)_scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    i2c_conf.master.clk_speed = 400000; // 400kHz

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));

    esp_err_t ret = i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install failed: 0x%x", ret);
        return false;
    }

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle);

    // rst_gpio_num is deliberately NC: selectI2cAddress() has already reset the
    // part, and letting the driver pulse RST again would re-latch the address
    // from a floating INT.
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_HRES,
        .y_max = CONFIG_LCD_VRES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = (gpio_num_t)_int,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller gt911");

    // A failed probe used to run into ESP_ERROR_CHECK and panic the board,
    // which is how a touch fault turned into a boot loop. Retry, then carry on
    // without touch.
    for (int attempt = 1; attempt <= 3; attempt++) {
        selectI2cAddress();

        ret = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp);
        if (ret == ESP_OK) return true;

        ESP_LOGE(TAG, "GT911 init attempt %d failed: 0x%x", attempt, ret);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    tp = NULL;
    return false;
}

bool gt911_touch::getTouch(uint16_t *x, uint16_t *y)
{
    // begin() leaves tp NULL when the controller never came up.
    if (tp == NULL) return false;

    esp_lcd_touch_read_data(tp);
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, x, y, touch_strength, &touch_cnt, 1);

    return touchpad_pressed;
}

void gt911_touch::set_rotation(uint8_t r){
if (tp == NULL) return;
switch(r){
    case 0:
        esp_lcd_touch_set_swap_xy(tp, false);   
        esp_lcd_touch_set_mirror_x(tp, false);
        esp_lcd_touch_set_mirror_y(tp, false);
        break;
    case 1:
        esp_lcd_touch_set_swap_xy(tp, false);
        esp_lcd_touch_set_mirror_x(tp, true);
        esp_lcd_touch_set_mirror_y(tp, true);
        break;
    case 2:
        esp_lcd_touch_set_swap_xy(tp, false);   
        esp_lcd_touch_set_mirror_x(tp, false);
        esp_lcd_touch_set_mirror_y(tp, false);
        break;
    case 3:
        esp_lcd_touch_set_swap_xy(tp, false);   
        esp_lcd_touch_set_mirror_x(tp, true);
        esp_lcd_touch_set_mirror_y(tp, true);
        break;
    }

}