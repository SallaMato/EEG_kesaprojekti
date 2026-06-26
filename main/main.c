#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "board_pins.h"
#include "ads131m08.h"

static const char *TAG = "MAIN";

static void configure_basic_pins(void)
{
    gpio_config_t out_conf = {
        .pin_bit_mask =
            (1ULL << PIN_ADS_CS) |
            (1ULL << PIN_ADS_SYNC_RESET),
        .mode = GPIO_MODE_OUTPUT,
    };

    gpio_config(&out_conf);

    gpio_set_level(PIN_ADS_CS, 1);
    gpio_set_level(PIN_ADS_SYNC_RESET, 1);

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_ADS_DRDY),
        .mode = GPIO_MODE_INPUT,
    };

    gpio_config(&in_conf);
}

static void start_ads_clkin(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = 1,
        .freq_hz = 8192000,
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num = PIN_ADS_CLKIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,
    };

    ledc_channel_config(&ch);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Boot");

    configure_basic_pins();
    start_ads_clkin();

    ads_init();
    ads_reset();

    ads_test_read_id();
    ads_test_write_readback_clock();
    ads_configure_ch2_ch3_gain();

    vTaskDelay(pdMS_TO_TICKS(3000));

    printf("HEADER,sample,t_us,ch2,ch3\n");

    int64_t next = esp_timer_get_time();

    while (1) {
        int64_t now = esp_timer_get_time();

        if (now >= next) {
            ads_stream_ch2_ch3_csv();
            next += 4000;
        } else {
            esp_rom_delay_us(100);
        }
    }
}