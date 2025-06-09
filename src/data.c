#include <zephyr/types.h>
#include <string.h>
#include "data.h"
#include "timer.h"
#include "spi.h"

stim_setting settings;
uint8_t ble_received_data[BLE_DATA_BUFFER_SIZE];
uint16_t ble_data_length;

void process_received_data(stim_setting *settings, uint8_t *ble_received_data, uint16_t ble_data_length) {
    if (ble_data_length == sizeof(stim_setting)) {
        memcpy(settings, ble_received_data, sizeof(stim_setting));
        // settings = (stim_setting *)ble_received_data;
        // Process the settings as needed
        // For example, you can print them or use them in your application logic
        printf("Received settings:\n");
        printf("DAC Amplitude: %u\n", settings->DAC_amplitude);
        printf("Pulse Width: %u us\n", settings->pulse_width);
        printf("Frequency: %u Hz\n", settings->frequency);
        if (settings->frequency > 0) {
            update_stim_frequency(settings->frequency);
        } else {
            printf("Warning: Received frequency is 0 Hz, timer not updated\n");
        }
        if (settings->pulse_width > 0) {
            update_pulse_width(settings->pulse_width);
        } else {
            printf("Warning: Received pulse width is 0 us, pulse width not updated\n");
        }
        update_dac1_amplitude(settings->DAC_amplitude);
        update_dac2_amplitude(settings->DAC_amplitude);
    } else {
        printf("Received data length mismatch: expected %zu, got %u\n",
               sizeof(stim_setting), ble_data_length);
    }
}