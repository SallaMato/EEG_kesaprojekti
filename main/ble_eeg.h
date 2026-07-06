#ifndef BLE_EEG_H
#define BLE_EEG_H

#include <stdint.h>

void ble_init(void);

void ble_send_sample(int32_t ch2,
                     int32_t ch3);

#endif