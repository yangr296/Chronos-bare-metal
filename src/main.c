/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include <nrfx_spim.h>
#include <nrfx_timer.h>
#include <zephyr/sys/atomic.h>

#include "BLE.h"
#include "spi.h"
#include "timer.h"

LOG_MODULE_REGISTER(mymain, LOG_LEVEL_DBG);
static void init_clock();
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

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

int main(void)
{
    #if defined(__ZEPHYR__)
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_IDX)), IRQ_PRIO_LOWEST,
                    NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_IDX), 0, 0);
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), IRQ_PRIO_LOWEST,
                    NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX), 0, 0);
    #endif

    init_clock();
    init_misc_pins();
    spi_init();
    timer_init();
    measurement_timer_init();
	int blink_status = 0;
	int err = 0;
    uint32_t experiment_counter = 0;
    
	configure_gpio();

	err = uart_init();
	if (err) {
		error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		//k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
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
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

static void init_clock() {
	// select the clock source: HFINT (high frequency internal oscillator) or HFXO (external 32 MHz crystal)
	NRF_CLOCK_S->HFCLKSRC = (CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos);

    // start the clock, and wait to verify that it is running
    NRF_CLOCK_S->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK_S->EVENTS_HFCLKSTARTED == 0);
    NRF_CLOCK_S->EVENTS_HFCLKSTARTED = 0;
}
