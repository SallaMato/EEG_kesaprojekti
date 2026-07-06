#pragma once

#include <stdint.h>
#include <stdbool.h>

//--------------------------------------------------
// Initialization
//--------------------------------------------------

void ads_init(void);
void ads_reset(void);

//--------------------------------------------------
// Self tests
//--------------------------------------------------

void ads_test_read_id(void);
void ads_test_write_readback_clock(void);

//--------------------------------------------------
// Configuration
//--------------------------------------------------

void ads_configure_ch2_ch3_gain(void);

//--------------------------------------------------
// Data acquisition
//--------------------------------------------------

bool ads_read_channels(int32_t *ch2, int32_t *ch3);

//--------------------------------------------------
// Optional low-level functions
//--------------------------------------------------

bool ads_read_data_frame(uint32_t words[]);
int32_t ads_sign_extend_24(uint32_t word24);