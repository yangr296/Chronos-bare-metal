#include <nrfx_timer.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "timer.h"
#include "spi.h"

static uint32_t timer_freq_hz = 0;  
static uint32_t main_event_time = 0;
static uint32_t event1_time = 0;
static uint32_t event2_time = 0;
static uint32_t event3_time = 0;

static atomic_t counter;            // test variable to record how many times the timer handler has been called 
static atomic_t error;
static atomic_t event1_error_max;
static atomic_t event2_error_max;
static atomic_t event3_error_max;
static atomic_t event0_error_counter;
static atomic_t event0_error_max;
static uint32_t prev_main_event_time = 0;
static nrfx_timer_t measurement_timer = NRFX_TIMER_INSTANCE(1); // Use a separate timer for measurements
static nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);; // Timer instance for the main timer
static void timer_handler(nrf_timer_event_t event_type, void * p_context);

void get_error_data(error_data *data) {
    data->event1_max = atomic_get(&event1_error_max);
    data->event2_max = atomic_get(&event2_error_max);
    data->event3_max = atomic_get(&event3_error_max);
    data->event0_error = atomic_get(&event0_error_counter);
    data->event0_max = atomic_get(&event0_error_max);
    data->myerror = atomic_get(&error);
    data->mycounter = atomic_get(&counter);
}

void timer_init(){
    atomic_set(&counter, 0);
    atomic_set(&error,0);
    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);    
    timer_freq_hz = base_frequency;
    printf("Timer frequency: %lu Hz\n", timer_freq_hz);
    
    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    config.p_context = &timer_inst;  // Pass timer instance as context for measurements
    nrfx_err_t status = nrfx_timer_init(&timer_inst, &config, timer_handler);
    nrfx_timer_clear(&timer_inst);
    if(status != NRFX_SUCCESS){
        printf("Timer initialization failed with error: %d\n", status);
    }
    uint32_t event1_ticks = nrfx_timer_us_to_ticks(&timer_inst, EVENT1_OFFSET_US);
    uint32_t event2_ticks = nrfx_timer_us_to_ticks(&timer_inst, (EVENT1_OFFSET_US + EVENT2_OFFSET_US));
    uint32_t event3_ticks = nrfx_timer_us_to_ticks(&timer_inst, (EVENT1_OFFSET_US + EVENT2_OFFSET_US + EVENT3_OFFSET_US));
    nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, 
                                nrfx_timer_us_to_ticks(&timer_inst, STIM_TIMER),
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL1, event1_ticks, 0, true);
    nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL2, event2_ticks, 0, true);
    nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL3, event3_ticks, 0, true);
    nrfx_timer_enable(&timer_inst);
    printf("Timer status: %s\n", nrfx_timer_is_enabled(&timer_inst) ? "enabled" : "disabled");
}

nrfx_timer_t measurement_timer_init() {
    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_BASE_FREQUENCY_GET(measurement_timer.p_reg));
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    nrfx_err_t err = nrfx_timer_init(&measurement_timer, &config, NULL); // No handler needed
    nrfx_timer_enable(&measurement_timer);
    return measurement_timer;
}

static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{   
    // Get reference to timer
    atomic_inc(&counter);
    //printf("Time handler count: %i \n", counter);
    nrfx_timer_t *timer_inst = (nrfx_timer_t *)p_context;
    uint32_t current_time;
    uint32_t current_max;
    uint32_t my_error;
    
    switch(event_type) {
        case NRF_TIMER_EVENT_COMPARE0:
            current_time = nrfx_timer_capture(&measurement_timer, NRF_TIMER_CC_CHANNEL0);
                
            if (prev_main_event_time > 0) {
                // Calculate actual interval duration
                uint32_t interval_ticks = current_time - prev_main_event_time;
                uint32_t expected_ticks = nrfx_timer_us_to_ticks(&measurement_timer, STIM_TIMER);
                uint32_t event0_error = abs(interval_ticks - expected_ticks);
                
                // Update statistics
                atomic_add(&event0_error_counter, event0_error);
                
                // Track maximum error
                current_max = atomic_get(&event0_error_max);
                if (event0_error > current_max) {
                    atomic_set(&event0_error_max, event0_error);
                }
            }
            prev_main_event_time = current_time;
            // Capture timestamp when main event occurs (after timer reset)
            main_event_time = nrfx_timer_capture(timer_inst, NRF_TIMER_CC_CHANNEL4);

            // Switch on 1.03
            nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1, 3));
            // SPI transaction on DAC 1
            // 100 us
            spi_write_dac1(dac1_buf_tx, dac1_buf_rx);
            break;
            
        case NRF_TIMER_EVENT_COMPARE1:
            // Capture timestamp when event 1 occurs
            current_time = nrfx_timer_capture(timer_inst, NRF_TIMER_CC_CHANNEL4);
            // Calculate elapsed time from main event
            uint32_t elapsed1_ticks = current_time - main_event_time;
            my_error = abs(elapsed1_ticks-EVENT1_OFFSET_US*timer_freq_hz/1000000);
            atomic_add(&error,my_error);
            current_max = atomic_get(&event1_error_max);
            if (my_error > current_max) {atomic_set(&event1_error_max, my_error);}

            // Switch off 1.03
            nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 3));
            // Switch on 1.00
            nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1, 0));
            // Switch on 1.01
            nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1, 1));
            // wait 10 us
            break;
            
        case NRF_TIMER_EVENT_COMPARE2:
            // Capture timestamp when event 2 occurs
            current_time = nrfx_timer_capture(timer_inst, NRF_TIMER_CC_CHANNEL4);
            // Calculate elapsed time from event 1
            my_error = abs(elapsed1_ticks-EVENT2_OFFSET_US*timer_freq_hz/1000000);
            atomic_add(&error, my_error);
            current_max = atomic_get(&event2_error_max);
            if (my_error > current_max) {atomic_set(&event2_error_max, my_error);}

            // Switch on 1.03
            nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1, 3));
            // SPI transaction on DAC2 
            // 100 us
            spi_write_dac2(dac2_buf_tx, dac2_buf_rx);
            break;
            
        case NRF_TIMER_EVENT_COMPARE3:
            // Capture timestamp when event 3 occurs
            current_time = nrfx_timer_capture(timer_inst, NRF_TIMER_CC_CHANNEL4);
            // Calculate elapsed time from event 2
            my_error = abs(elapsed1_ticks-EVENT3_OFFSET_US*timer_freq_hz/1000000);
            atomic_add(&error,my_error);
            current_max = atomic_get(&event3_error_max);
            if (my_error > current_max) {atomic_set(&event3_error_max, my_error);}
            // Switch off 1.03
            nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 3));
            // Switch on 1.00
            nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1, 0));
            // Switch on 1.01
            nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1, 1));
            // wait 10 us
            break;
    }
}