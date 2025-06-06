#ifndef DATA_H
#define DATA_H
#include <zephyr/types.h>

typedef struct{
    uint16_t DAC_amplitude;     //binary
    uint16_t pulse_width;       // us
    uint16_t frequency;         // Hz
} stim_setting;

#define BLE_DATA_BUFFER_SIZE (sizeof(stim_setting)) 
extern uint8_t ble_received_data[];
extern uint16_t ble_data_length;
extern stim_setting settings;

void process_received_data(stim_setting *settings, uint8_t *ble_received_data, uint16_t ble_data_length);
#endif // DATA_H