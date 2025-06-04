#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <nrfx_spim.h>
#include <nrfx_timer.h>
#include <nrfx_log.h>
#include <zephyr/sys/atomic.h>
#include "spi.h"
#include "timer.h"

static void init_clock();

// keeps track of whether stimulation is currently ongoing
bool stimming = false;

nrfx_err_t status;

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

int main(void){    
    #if defined(__ZEPHYR__)
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_IDX)), IRQ_PRIO_LOWEST,
                    NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_IDX), 0, 0);
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), IRQ_PRIO_LOWEST,
                    NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX), 0, 0);
    #endif
    init_clock();
    init_misc_pins();
    (void)status;
    spi_init();
    timer_init();
    measurement_timer_init();
    uint32_t experiment_counter = 0;
    while(1) {
        //NRFX_EXAMPLE_LOG_PROCESS();
        k_msleep(10000);
        experiment_counter += 10;
        error_data my_error_data;
        get_error_data(&my_error_data);
        printf("Counter: %i Elapsed: %is\nEvent0 running error: %lu avg error: %lu max error: %lu\nEvents1-3 running error: %lu avg error: %lu max error: %lu, %lu, %lu\n", 
               my_error_data.mycounter, 
               experiment_counter,
               my_error_data.event0_error,
               (my_error_data.event0_error/my_error_data.mycounter), 
               my_error_data.event0_max,
               my_error_data.myerror,
               (my_error_data.myerror/my_error_data.mycounter),
               my_error_data.event1_max,
               my_error_data.event2_max,
               my_error_data.event3_max);
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




