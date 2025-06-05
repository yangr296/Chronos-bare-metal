#ifndef BLE_H
#define BLE_H

#include <zephyr/types.h>
#include <zephyr/device.h>

#define LOG_MODULE_NAME peripheral_uart

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif // CONFIG_UART_ASYNC_ADAPTER

extern struct k_sem ble_init_ok;
extern struct k_work_delayable uart_work;
extern const struct device *uart;
extern struct k_work adv_work;
extern struct bt_conn *current_conn;
extern struct bt_conn *auth_conn;
extern const struct bt_data ad[];
extern const struct bt_data sd[];
extern const size_t ad_len;
extern const size_t sd_len;
struct uart_data_t {
    void *fifo_reserved;
    uint8_t data[UART_BUF_SIZE];
    uint16_t len;
};

void uart_work_handler(struct k_work *item);
bool uart_test_async_api(const struct device *dev);
void adv_work_handler(struct k_work *work);
void advertising_start(void);
void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);
void recycled_cb(void);
void configure_gpio(void);
void error(void);
void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len);
void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);
int uart_init(void);
void ble_write_thread(void);

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
void security_changed(struct bt_conn *conn, bt_security_t level,
                 enum bt_security_err err);
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
void auth_passkey_display(struct bt_conn *conn, unsigned int passkey);
void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey);
void auth_cancel(struct bt_conn *conn);
void pairing_complete(struct bt_conn *conn, bool bonded);
void pairing_failed(struct bt_conn *conn, enum bt_security_err reason);
extern struct bt_conn_auth_cb conn_auth_callbacks;
extern struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#else
extern struct bt_conn_auth_cb conn_auth_callbacks;
extern struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
void num_comp_reply(bool accept);
void button_changed(uint32_t button_state, uint32_t has_changed);
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

#endif /* BLE_H */

