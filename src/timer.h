#ifndef TIMER_H
#define TIMER_H

#include <nrfx_timer.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define TIMER_INST_IDX 0
//This is the time between stim
#define STIM_TIMER 4000000

// This is the time between SPI transac on DAC1 and switching 1.03 off
#define EVENT1_OFFSET_US 1000000  // x1: Time after main event
// This is the time between switching 1.03 off and SPI transac on DAC2 
#define EVENT2_OFFSET_US 1000000  // x2: Time after EVENT1
// This is the time between SPI transac on DAC2 and switching 1.03 off
#define EVENT3_OFFSET_US 1000000 // x3: Time after EVENT2

typedef struct {
    uint32_t event1_max;
    uint32_t event2_max;
    uint32_t event3_max;
    uint32_t event0_error;
    uint32_t event0_max;
    uint32_t myerror;
    uint32_t mycounter;
} error_data;

void timer_init();
void get_error_data(error_data *data);
nrfx_timer_t measurement_timer_init();
#endif