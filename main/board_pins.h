#pragma once

#include "driver/gpio.h"

// ADS131M08 <-> ESP32-S3
#define PIN_ADS_SYNC_RESET   GPIO_NUM_21
#define PIN_ADS_DRDY         GPIO_NUM_13
#define PIN_ADS_CS           GPIO_NUM_14
#define PIN_ADS_DIN_MOSI     GPIO_NUM_10
#define PIN_ADS_DOUT_MISO    GPIO_NUM_11
#define PIN_ADS_SCLK         GPIO_NUM_12
#define PIN_ADS_CLKIN        GPIO_NUM_18

// Debug spare pins
#define PIN_DEBUG_IO9        GPIO_NUM_9
#define PIN_DEBUG_IO15       GPIO_NUM_15
#define PIN_DEBUG_IO16       GPIO_NUM_16

// ESP control / fallback
#define PIN_BOOT             GPIO_NUM_0
#define PIN_UART_TXD0        GPIO_NUM_43
#define PIN_UART_RXD0        GPIO_NUM_44