#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "board_pins.h"
#include "ads131m08.h"
#include "ble_eeg.h"
#include "nvs_flash.h"

static const char *TAG = "EEG_BOARD";

// 250 SPS
#define ADS_STREAM_PERIOD_US    4000

//----------------------------------------------------------------------
// GPIO
//----------------------------------------------------------------------

static void configure_basic_pins(void)
{
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

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_ADS_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&in_conf));
}

//----------------------------------------------------------------------
// ADS CLKIN
//----------------------------------------------------------------------

static void start_ads_clkin(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = (ledc_timer_bit_t)1,
        .freq_hz = 8192000,
        .clk_cfg = LEDC_USE_APB_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = PIN_ADS_CLKIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

//----------------------------------------------------------------------
// Main
//----------------------------------------------------------------------

void app_main(void)
{
    ESP_LOGI(TAG, "Boot");

    esp_err_t ret = nvs_flash_init();

if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
    ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
{
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
}

    configure_basic_pins();

    start_ads_clkin();
    ESP_LOGI(TAG, "ADS CLKIN started");

    ads_init();
    ads_reset();

    ads_test_read_id();
    ads_test_write_readback_clock();
    ads_configure_ch2_ch3_gain();

    ble_init();

    ESP_LOGI(TAG, "Startup complete");

    vTaskDelay(pdMS_TO_TICKS(3000));

    printf("HEADER,sample,t_us,ch2_raw,ch3_raw\n");

    uint32_t sample = 0;

    int64_t next_sample_time = esp_timer_get_time();

    while (1)
    {
        int64_t now = esp_timer_get_time();

        if (now >= next_sample_time)
        {
            int32_t ch2;
            int32_t ch3;

            if (ads_read_channels(&ch2, &ch3))
            {
                printf("DATA,%lu,%lld,%ld,%ld\n",
                       (unsigned long)sample,
                       (long long)esp_timer_get_time(),
                       (long)ch2,
                       (long)ch3);

                ble_send_sample(ch2, ch3);

                sample++;
            }

            next_sample_time += ADS_STREAM_PERIOD_US;
        }
        else
        {
            esp_rom_delay_us(100);
        }
    }
}