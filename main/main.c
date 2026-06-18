#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

#include "board_pins.h"
#include "esp_rom_sys.h"

static const char *TAG = "EEG_BOARD";

#define ADS_SPI_HOST        SPI2_HOST
#define ADS_SPI_FREQ_HZ     100000
#define ADS_WORD_BYTES      3
#define ADS_FRAME_WORDS     10
#define ADS_FRAME_BYTES     (ADS_WORD_BYTES * ADS_FRAME_WORDS)

#define ADS_REG_ID          0x00

static spi_device_handle_t ads_spi = NULL;

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

static void ads_reset_pulse(void)
{
    ESP_LOGI(TAG, "Resetting ADS131M08");

    gpio_set_level(PIN_ADS_SYNC_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(PIN_ADS_SYNC_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ADS reset released");
}

static void ads_spi_init(void)
{
    ESP_LOGI(TAG, "Initializing SPI bus");

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_ADS_DIN_MOSI,
        .miso_io_num = PIN_ADS_DOUT_MISO,
        .sclk_io_num = PIN_ADS_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ADS_FRAME_BYTES,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(ADS_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = ADS_SPI_FREQ_HZ,
        .mode = 1,
        .spics_io_num = -1, // CS handled manually as GPIO
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(ADS_SPI_HOST, &devcfg, &ads_spi));

    ESP_LOGI(TAG, "SPI initialized: SCLK=%d Hz, mode=1", ADS_SPI_FREQ_HZ);
}

static uint32_t ads_rx_word24(const uint8_t *rx, int word_index)
{
    int i = word_index * 3;

    return ((uint32_t)rx[i] << 16) |
           ((uint32_t)rx[i + 1] << 8) |
           ((uint32_t)rx[i + 2]);
}

static void ads_put_word24(uint8_t *tx, int word_index, uint32_t word24)
{
    int i = word_index * 3;

    tx[i]     = (word24 >> 16) & 0xFF;
    tx[i + 1] = (word24 >> 8) & 0xFF;
    tx[i + 2] = word24 & 0xFF;
}

static void ads_transfer_frame(const uint8_t *tx, uint8_t *rx)
{
    spi_transaction_t trans = {
        .length = ADS_FRAME_BYTES * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    gpio_set_level(PIN_ADS_CS, 0);
    esp_rom_delay_us(2);

    ESP_ERROR_CHECK(spi_device_transmit(ads_spi, &trans));

    esp_rom_delay_us(2);
    gpio_set_level(PIN_ADS_CS, 1);
    esp_rom_delay_us(2);
}

static uint16_t ads_make_rreg_command(uint8_t reg_addr, uint8_t reg_count)
{
    return 0xA000 |
           ((reg_addr & 0x3F) << 7) |
           ((reg_count - 1) & 0x7F);
}

static uint16_t ads_read_single_register(uint8_t reg_addr)
{
    uint8_t tx1[ADS_FRAME_BYTES] = {0};
    uint8_t rx1[ADS_FRAME_BYTES] = {0};

    uint8_t tx2[ADS_FRAME_BYTES] = {0};
    uint8_t rx2[ADS_FRAME_BYTES] = {0};

    uint16_t rreg_cmd = ads_make_rreg_command(reg_addr, 1);

    ads_put_word24(tx1, 0, ((uint32_t)rreg_cmd) << 8);

    ESP_LOGI(TAG, "Sending RREG command for register 0x%02X: 0x%04X", reg_addr, rreg_cmd);

    ads_transfer_frame(tx1, rx1);
    ads_transfer_frame(tx2, rx2);

    for (int i = 0; i < ADS_FRAME_WORDS; i++) {
    uint32_t w = ads_rx_word24(rx2, i);
    ESP_LOGI(TAG, "RX2 word[%d] = 0x%06lX", i, (unsigned long)w);
}

    uint32_t response_word24 = ads_rx_word24(rx2, 0);
    uint16_t reg_value = (response_word24 >> 8) & 0xFFFF;

    ESP_LOGI(TAG, "RREG response word24 = 0x%06lX", (unsigned long)response_word24);
    ESP_LOGI(TAG, "Register 0x%02X value = 0x%04X", reg_addr, reg_value);

    return reg_value;
}

static void ads_test_read_id(void)
{
    ESP_LOGI(TAG, "Reading ADS131M08 ID register");

    uint16_t id = ads_read_single_register(ADS_REG_ID);

    ESP_LOGI(TAG, "ADS ID register = 0x%04X", id);

    if ((id & 0xFF00) == 0x2800) {
        ESP_LOGI(TAG, "ADS ID looks OK: expected upper byte 0x28");
    } else {
        ESP_LOGW(TAG, "ADS ID unexpected. Expected something like 0x28xx.");
    }
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

    ads_spi_init();

    ads_reset_pulse();

   while (1) {
    ads_test_read_id();
    ESP_LOGI(TAG, "Alive");
    vTaskDelay(pdMS_TO_TICKS(2000));
}

}
/*void app_main(void)
{
    ESP_LOGI(TAG, "GPIO10 manual toggle test");

    gpio_config_t test_conf = {
        .pin_bit_mask = (1ULL << PIN_ADS_DIN_MOSI),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&test_conf));

    while (1) {
        gpio_set_level(PIN_ADS_DIN_MOSI, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(PIN_ADS_DIN_MOSI, 0);
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "GPIO10 toggled");
    }
}*/
