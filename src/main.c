#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <nrfx_spim.h>
#include <nrfx_timer.h>
#include <nrfx_log.h>
#include <zephyr/sys/atomic.h>
#include "spi.h"

#define TIMER_INST_IDX 0

//This is the time between stim
#define STIM_TIMER 4000000

// This is the time between SPI transac on DAC1 and switching 1.03 off
#define EVENT1_OFFSET_US 1000000  // x1: Time after main event
// This is the time between switching 1.03 off and SPI transac on DAC2 
#define EVENT2_OFFSET_US 1000000  // x2: Time after EVENT1
// This is the time between SPI transac on DAC2 and switching 1.03 off
#define EVENT3_OFFSET_US 1000000 // x3: Time after EVENT2


// define pins and ports for dac slave:         P0.25
#define DAC1_CS_PIN 16  // P0.16
#define DAC2_CS_PIN 26  // P0.26
#define DAC_TX_LEN			2
#define DAC_RX_LEN			2
//! might need separate buffers for each transaction
uint8_t dac1_buf_rx[DAC_RX_LEN];
uint8_t dac1_buf_tx[DAC_TX_LEN] = {0x52, 0x53};
uint8_t dac2_buf_rx[DAC_RX_LEN];
uint8_t dac2_buf_tx[DAC_TX_LEN] = {0x54, 0x55};

static void init_clock();

// keeps track of whether stimulation is currently ongoing
bool stimming = false;

nrfx_err_t status;

// These are used to evaluate precision 
static uint32_t main_event_time = 0;
static uint32_t event1_time = 0;
static uint32_t event2_time = 0;
static uint32_t event3_time = 0;

static uint32_t timer_freq_hz = 0;      // record the frequency of the timer 
static atomic_t counter;            // test variable to record how many times the timer handler has been called 
static atomic_t error;
static atomic_t event1_error_max;
static atomic_t event2_error_max;
static atomic_t event3_error_max;
static atomic_t event0_error_counter;
static atomic_t event0_error_max;
static uint32_t prev_main_event_time = 0;
static nrfx_timer_t measurement_timer;

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
            spi_write_dac1(dac1_buf_tx, dac2_buf_rx);
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

static void init_misc_pins(void) {
    // Configure P0.16 as output (DAC1 CS)
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 16));
    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0, 16));  // Set high (inactive)
    
    // Configure P0.26 as output (DAC2 CS)
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 26));
    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0, 26));  // Set high (inactive)
    
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 3));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 3)); // set low

    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 0));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 0)); // set low

    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 1));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 1)); // set low
}

static void timer_init(nrfx_timer_t timer_inst){
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

static nrfx_timer_t measurement_timer_init() {
    // Initialize a second timer that doesn't auto-clear
    nrfx_timer_t my_timer = NRFX_TIMER_INSTANCE(1); // Use another timer instance
    
    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_BASE_FREQUENCY_GET(my_timer.p_reg));
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    nrfx_err_t err = nrfx_timer_init(&my_timer, &config, NULL); // No handler needed
    nrfx_timer_enable(&my_timer);
    return my_timer;
}

int main(void){    
    #if defined(__ZEPHYR__)
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_IDX)), IRQ_PRIO_LOWEST,
                    NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_IDX), 0, 0);
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), IRQ_PRIO_LOWEST,
                    NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX), 0, 0);
    #endif
    init_clock();
    atomic_set(&counter, 0);
    atomic_set(&error,0);
    init_misc_pins();
    (void)status;
    spi_init();
    nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);
    timer_init(timer_inst);
    measurement_timer = measurement_timer_init();
    uint32_t experiment_counter = 0;
    while(1) {
        //NRFX_EXAMPLE_LOG_PROCESS();
        k_msleep(10000);
        experiment_counter += 10;
        uint32_t mycounter = atomic_get(&counter);
        uint32_t myerror = atomic_get(&error);
        // printf("Counter: %i, Average error: %lu, Running Error: %lu\n", mycounter, (myerror/mycounter), myerror);
        uint32_t event0_error = atomic_get(&event0_error_counter);
        uint32_t event0_max = atomic_get(&event0_error_max);
        uint32_t event1_max = atomic_get(&event1_error_max);
        uint32_t event2_max = atomic_get(&event2_error_max);
        uint32_t event3_max = atomic_get(&event3_error_max);

        printf("Counter: %i Elapsed: %is\nEvent0 running error: %lu avg error: %lu max error: %lu\nEvents1-3 running error: %lu avg error: %lu max error: %lu, %lu, %lu\n", 
               mycounter, 
               experiment_counter,
               event0_error,
               (event0_error/mycounter), 
               event0_max,
               myerror,
               (myerror/mycounter),
               event1_max,
               event2_max,
               event3_max);
    }
    return 0;
}



static void init_clock() {
	// select the clock source: HFINT (high frequency internal oscillator) or HFXO (external 32 MHz crystal)
	NRF_CLOCK_S->HFCLKSRC = (CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos);

    // start the clock, and wait to verify that it is running
    NRF_CLOCK_S->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK_S->EVENTS_HFCLKSTARTED == 0);
    NRF_CLOCK_S->EVENTS_HFCLKSTARTED = 0;
}




