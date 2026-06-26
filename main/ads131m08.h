#pragma once

#include <stdint.h>
#include <stdbool.h>

void ads_init(void);
void ads_reset(void);

void ads_test_read_id(void);
void ads_test_write_readback_clock(void);

void ads_configure_ch2_ch3_gain(void);

bool ads_read_data_frame(uint32_t words[]);
int32_t ads_sign_extend_24(uint32_t word24);

void ads_stream_ch2_ch3_csv(void);