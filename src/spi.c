#include <nrfx_spim.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <hal/nrf_gpio.h>
#include "spi.h"

static nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);
static void spim_handler(nrfx_spim_evt_t const * p_event, void * p_context);
uint8_t dac1_buf_tx[DAC_TX_LEN] = {0x52, 0x53};
uint8_t dac2_buf_tx[DAC_TX_LEN] = {0x54, 0x55};
uint8_t dac1_buf_rx[DAC_RX_LEN];
uint8_t dac2_buf_rx[DAC_RX_LEN];

void cs_select(uint32_t pin_number) {
    nrf_gpio_pin_clear(pin_number);  // Drive CS low (active)
}

void cs_deselect(uint32_t pin_number) {
    nrf_gpio_pin_set(pin_number);     // Drive CS high (inactive)
}

void spi_write_dac1(uint8_t *tx_data, uint8_t *rx_data) {
    // Select DAC1
    cs_select(DAC1_CS_PIN);
    memset(rx_data, 0, DAC_RX_LEN); // Clear RX buffer
    // Prepare transfer descriptor
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_data, DAC_TX_LEN, rx_data, DAC_RX_LEN);
    
    // Perform the transfer
    nrfx_err_t err = nrfx_spim_xfer(&spim_inst, &xfer_desc, 0);
    if(err != NRFX_SUCCESS){
        printf("SPI ERROR\n");
    }
    cs_deselect(DAC1_CS_PIN);
}

void spi_write_dac2(uint8_t *tx_data, uint8_t *rx_data) {
    // Select DAC1
    cs_select(DAC2_CS_PIN);
    memset(rx_data, 0, DAC_RX_LEN); // Clear RX buffer
    // Prepare transfer descriptor
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_data, DAC_TX_LEN, rx_data, DAC_RX_LEN);
    
    // Perform the transfer
    nrfx_err_t err = nrfx_spim_xfer(&spim_inst, &xfer_desc, 0);
    if(err != NRFX_SUCCESS){
        printf("SPI ERROR\n");
    }
    cs_deselect(DAC2_CS_PIN);
}
void spi_init(){
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SCK_PIN,
                                                              MOSI_PIN,
                                                              MISO_PIN,
                                                              NRF_SPIM_PIN_NOT_CONNECTED);

    spim_config.frequency = 8000000;
    nrfx_err_t status = nrfx_spim_init(&spim_inst, &spim_config, spim_handler, NULL);
    if (status == NRFX_SUCCESS) {
        printf("SPI initialized successfully on SPIM%d\n", SPIM_INST_IDX);
        printf("  SCK: P%d.%02d\n", (SCK_PIN >> 5), (SCK_PIN & 0x1F));
        printf("  MOSI: P%d.%02d\n", (MOSI_PIN >> 5), (MOSI_PIN & 0x1F)); 
        printf("  MISO: P%d.%02d\n", (MISO_PIN >> 5), (MISO_PIN & 0x1F));
    } else {
        printf("SPI initialization failed with error: %d\n", status);
    }
}

static void spim_handler(nrfx_spim_evt_t const * p_event, void * p_context){
    if (p_event->type == NRFX_SPIM_EVENT_DONE){
        printf("Message received: %02X\n", p_event->xfer_desc.p_rx_buffer);
    }
}

