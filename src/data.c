#include <zephyr/types.h>
#include "data.h"

stim_setting settings;
uint8_t ble_received_data[BLE_DATA_BUFFER_SIZE];
uint16_t ble_data_length;

void process_received_data(stim_setting *settings, uint8_t *ble_received_data, uint16_t ble_data_length) {
    if (ble_data_length == sizeof(stim_setting)) {
        settings = (stim_setting *)ble_received_data;
        // Process the settings as needed
        // For example, you can print them or use them in your application logic
        printf("Received settings:\n");
        printf("DAC Amplitude: %u\n", settings->DAC_amplitude);
        printf("Pulse Width: %u us\n", settings->pulse_width);
        printf("Frequency: %u Hz\n", settings->frequency);
    } else {
        printf("Received data length mismatch: expected %zu, got %u\n",
               sizeof(stim_setting), ble_data_length);
    }
}