#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "board_pins.h"

static const char *TAG = "EEG_BOARD";

static void configure_basic_pins(void)
{
    // ADS CS idle high, SYNC/RESET idle high
    gpio_config_t out_conf = {
        .pin_bit_mask =
            (1ULL << PIN_ADS_CS) |
            (1ULL << PIN_ADS_SYNC_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_conf));

    gpio_set_level(PIN_ADS_CS, 1);
    gpio_set_level(PIN_ADS_SYNC_RESET, 1);

    // ADS DRDY input
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_ADS_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_conf));
}

static void start_ads_clkin(void)
{
    // ADS131M08 master clock: 8.192 MHz, about 50% duty
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = (ledc_timer_bit_t)1,
        .freq_hz = 8192000,
        .clk_cfg = LEDC_USE_APB_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = PIN_ADS_CLKIN,        // GPIO18
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,                        // 50% with 1-bit resolution
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

static void ads_reset_pulse(void)
{
    ESP_LOGI(TAG, "Resetting ADS131M08");

    // SYNC/RESET is active low
    gpio_set_level(PIN_ADS_SYNC_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(PIN_ADS_SYNC_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ADS reset released");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Boot ok");
    ESP_LOGI(TAG, "ADS pins:");
    ESP_LOGI(TAG, "SYNC/RESET = GPIO%d", PIN_ADS_SYNC_RESET);
    ESP_LOGI(TAG, "DRDY       = GPIO%d", PIN_ADS_DRDY);
    ESP_LOGI(TAG, "CS         = GPIO%d", PIN_ADS_CS);
    ESP_LOGI(TAG, "MOSI/DIN   = GPIO%d", PIN_ADS_DIN_MOSI);
    ESP_LOGI(TAG, "MISO/DOUT  = GPIO%d", PIN_ADS_DOUT_MISO);
    ESP_LOGI(TAG, "SCLK       = GPIO%d", PIN_ADS_SCLK);
    ESP_LOGI(TAG, "CLKIN      = GPIO%d", PIN_ADS_CLKIN);

    configure_basic_pins();

    start_ads_clkin();
    ESP_LOGI(TAG, "ADS CLKIN started on GPIO%d", PIN_ADS_CLKIN);

    ads_reset_pulse();

    while (1) {
        ESP_LOGI(TAG, "Alive");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}