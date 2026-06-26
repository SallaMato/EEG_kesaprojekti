#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "ads131m08.h"
#include "board_pins.h"

static const char *TAG = "ADS131M08";

// =================== CONFIG ===================

#define ADS_SPI_HOST        SPI2_HOST
#define ADS_SPI_FREQ_HZ     1000000

#define ADS_WORD_BYTES      3
#define ADS_FRAME_WORDS     10
#define ADS_FRAME_BYTES     (ADS_WORD_BYTES * ADS_FRAME_WORDS)

#define ADS_REG_ID          0x00
#define ADS_REG_CLOCK       0x03
#define ADS_REG_GAIN1       0x04

#define ADS_CH2_WORD_INDEX  3
#define ADS_CH3_WORD_INDEX  4

#define ADS_PGA_GAIN_CODE_TEST 2
#define ADS_TEST_PGA_GAIN     4

static spi_device_handle_t ads_spi = NULL;

// =================== LOW-LEVEL ===================

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

// =================== SPI ===================

static void ads_spi_init(void)
{
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
        .spics_io_num = -1,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(ADS_SPI_HOST, &devcfg, &ads_spi));
}

// =================== REGISTER ===================

static uint16_t ads_make_rreg_command(uint8_t reg_addr, uint8_t reg_count)
{
    return 0xA000 |
           ((reg_addr & 0x3F) << 7) |
           ((reg_count - 1) & 0x7F);
}

static uint16_t ads_make_wreg_command(uint8_t reg_addr, uint8_t reg_count)
{
    return 0x6000 |
           ((reg_addr & 0x3F) << 7) |
           ((reg_count - 1) & 0x7F);
}

static uint16_t ads_read_single_register(uint8_t reg_addr)
{
    uint8_t tx1[ADS_FRAME_BYTES] = {0};
    uint8_t rx1[ADS_FRAME_BYTES] = {0};

    uint8_t tx2[ADS_FRAME_BYTES] = {0};
    uint8_t rx2[ADS_FRAME_BYTES] = {0};

    uint16_t cmd = ads_make_rreg_command(reg_addr, 1);

    ads_put_word24(tx1, 0, ((uint32_t)cmd) << 8);

    ads_transfer_frame(tx1, rx1);
    ads_transfer_frame(tx2, rx2);

    uint32_t resp = ads_rx_word24(rx2, 0);
    return (resp >> 8) & 0xFFFF;
}

static void ads_write_single_register(uint8_t reg_addr, uint16_t value)
{
    uint8_t tx1[ADS_FRAME_BYTES] = {0};
    uint8_t tx2[ADS_FRAME_BYTES] = {0};

    uint16_t cmd = ads_make_wreg_command(reg_addr, 1);

    ads_put_word24(tx1, 0, ((uint32_t)cmd) << 8);
    ads_put_word24(tx1, 1, ((uint32_t)value) << 8);

    ads_transfer_frame(tx1, NULL);
    ads_transfer_frame(tx2, NULL);
}

// =================== PUBLIC API ===================

void ads_init(void)
{
    ads_spi_init();
}

void ads_reset(void)
{
    gpio_set_level(PIN_ADS_SYNC_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_ADS_SYNC_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void ads_test_read_id(void)
{
    uint16_t id = ads_read_single_register(ADS_REG_ID);
    ESP_LOGI(TAG, "ADS ID = 0x%04X", id);
}

void ads_test_write_readback_clock(void)
{
    uint16_t before = ads_read_single_register(ADS_REG_CLOCK);
    ads_write_single_register(ADS_REG_CLOCK, before);
    uint16_t after = ads_read_single_register(ADS_REG_CLOCK);

    ESP_LOGI(TAG, "CLOCK before=0x%04X after=0x%04X", before, after);
}

void ads_configure_ch2_ch3_gain(void)
{
    uint16_t gain =
        (ADS_PGA_GAIN_CODE_TEST << 12) |
        (ADS_PGA_GAIN_CODE_TEST << 8);

    ads_write_single_register(ADS_REG_GAIN1, gain);
}

// =================== DATA ===================

int32_t ads_sign_extend_24(uint32_t word24)
{
    if (word24 & 0x800000) {
        return (int32_t)(word24 | 0xFF000000);
    }
    return (int32_t)word24;
}

static bool ads_wait_drdy_low(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();

    while (gpio_get_level(PIN_ADS_DRDY) != 0) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

bool ads_read_data_frame(uint32_t words[])
{
    uint8_t tx[ADS_FRAME_BYTES] = {0};
    uint8_t rx[ADS_FRAME_BYTES] = {0};

    if (!ads_wait_drdy_low(1000)) {
        return false;
    }

    ads_transfer_frame(tx, rx);

    for (int i = 0; i < ADS_FRAME_WORDS; i++) {
        words[i] = ads_rx_word24(rx, i);
    }

    return true;
}

void ads_stream_ch2_ch3_csv(void)
{
    static uint32_t sample_index = 0;

    uint32_t words[ADS_FRAME_WORDS];

    if (!ads_read_data_frame(words)) {
        return;
    }

    int64_t t = esp_timer_get_time();

    int32_t ch2 = ads_sign_extend_24(words[ADS_CH2_WORD_INDEX]);
    int32_t ch3 = ads_sign_extend_24(words[ADS_CH3_WORD_INDEX]);

    printf("DATA,%lu,%lld,%ld,%ld\n",
           sample_index++, t, ch2, ch3);
}