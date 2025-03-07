#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "nrf_example.h"
#include <nrfx_spim.h>
#include <nrfx_timer.h>
#include <nrfx_log.h>
#include "nrfx_example.h"

//! USE NO-SYS-BUILD 
// using the controller SPIM4 at 16 MHz frequency
// SPIM4 is the only onboard spi controller capable of running at 16 MHz
// other available frequencies: M8, M4, M2, M1, K500, K250
#define SPI_1 				NRF_SPIM0_S
#define SPI_1_FREQUENCY 	SPIM_FREQUENCY_FREQUENCY_M8 // 16 MHz

#define TIMER_INST_IDX 0
#define TIME_TO_WAIT_MS 500

// define the pins and ports that control the clock, miso, and mosi pins on SPI_1
//      clock:      P0.08
//      miso:       P0.09
//      mosi:       P0.10
#define SPI_1_CLK_PIN_NUM     	10
#define SPI_1_MOSI_PIN_NUM    	11
#define SPI_1_MISO_PIN_NUM    	12

#define SPI_1_CLK_PORT    	NRF_P1_S
#define SPI_1_MOSI_PORT   	NRF_P1_S
#define SPI_1_MISO_PORT   	NRF_P1_S

#define SPI_1_CLK_PORT_NUM    	1
#define SPI_1_MOSI_PORT_NUM   	1
#define SPI_1_MISO_PORT_NUM   	1

// all of these slave devices are placed on the same controller: SPI_1 = SPIM
#define SENSE_SPI_CONTROLLER    SPI_1
#define DAC_SPI_CONTROLLER      SPI_1
#define SWITCH_SPI_CONTROLLER   SPI_1

// define pin and ports for sensing slave:      P0.07
#define SENSE_CS_PIN_NUM    7
#define SENSE_CS_PORT_NUM   0
#define SENSE_CS_PORT       NRF_P0_S
#define SENSE_TX_LEN		1
#define SENSE_RX_LEN		1
uint8_t sense_buf_rx[SENSE_RX_LEN];
uint8_t sense_buf_tx[SENSE_TX_LEN] = {0xFF};

// define pins and ports for dac slave:         P0.25
#define DAC_CS_PIN_NUM      25
#define DAC_CS_PORT_NUM     0
#define DAC_CS_PORT         NRF_P0_S
#define DAC_TX_LEN			2
#define DAC_RX_LEN			2
uint8_t dac_buf_rx[DAC_RX_LEN];
uint8_t dac_buf_tx[DAC_TX_LEN] = {0x55, 0x55};

// define pins and ports for switching slave:   P0.26
#define SWITCH_CS_PIN_NUM   26
#define SWITCH_CS_PORT_NUM  0
#define SWITCH_CS_PORT      NRF_P0_S
#define SWITCH_TX_LEN		1
#define SWITCH_RX_LEN		1
uint8_t switch_buf_rx[SWITCH_RX_LEN];
uint8_t switch_buf_tx[SWITCH_TX_LEN] = {0x01};

// data which is to be sent on the mosi line to the slave devices
// note that there is a different variable not only for each device, but each type of transmission
// I tried to change the value only between transmissions, but the data will remain the same unless pointer is changed
uint8_t sense_on = 0xFF;
uint8_t sense_off = 0x00;
uint16_t dac_high = 0xFFFF;
uint16_t dac_off = 0x5555;
uint16_t dac_low = 0x0000;
uint8_t switch_short = 0xFF;
uint8_t switch_open = 0x00;

#define SPI_SET_RX_BUFFER(controller, rxptr, len)	controller->RXD.PTR = (uint32_t)rxptr;	\
													controller->RXD.MAXCNT = len

#define SPI_SET_TX_BUFFER(controller, txptr, len)	controller->TXD.PTR = (uint32_t)txptr; 	\
													controller->TXD.MAXCNT = len				

// complete a full spi transaction - equivalent to calling START_SPI_TRANS and WAIT_SPI_TRANS sequentially
#define TRANSMIT_SPI(controller, pin, port)     port->OUTCLR = (1 << pin); \
											    controller->TASKS_START = 1; \
                                                while(!controller->EVENTS_END); \
												controller->EVENTS_END = 0; \
												port->OUTSET = (1 << pin)

// helper macros to enable and disable a particular spi controller
#define ENABLE_SPI(controller) controller->ENABLE = SPIM_ENABLE_ENABLE_Enabled; while(controller->ENABLE != SPIM_ENABLE_ENABLE_Enabled)
#define DISABLE_SPI(controller) controller->ENABLE = SPIM_ENABLE_ENABLE_Disabled

// set the interval for spi stimulation
// e.g. 1000 us interval corresponds to a stimulation frequency of 1 / 1000 us =  1 kHz
#define SPI_SAMPLE_PERIOD_US 1000
#define SPI_SAMPLE_PERIOD K_USEC(SPI_SAMPLE_PERIOD_US)

static void init_clock();

// callback for our K_TIMER - triggers a SPI stimulation
void spi_timer_handler(struct k_timer * dummy);

// keeps track of whether stimulation is currently ongoing
bool stimming = false;

//TODO: change this to nrfx
//!K_TIMER_DEFINE(spi_timer, spi_timer_handler, NULL);

/** @brief Symbol specifying SPIM instance to be used. */
#define SPIM_INST_IDX 1

/** @brief Symbol specifying pin number for MOSI. */
#define MOSI_PIN 7

/** @brief Symbol specifying pin number for MISO. */
#define MISO_PIN 25

/** @brief Symbol specifying pin number for SCK. */
#define SCK_PIN 26

nrfx_err_t status;
nrfx_spim_t spim_inst;

//TODO replace this with the spi_timer_handler 
static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    if(event_type == NRF_TIMER_EVENT_COMPARE0)
    {
        char * p_msg = p_context;
        NRFX_LOG_INFO("Timer finished. Context passed to the handler: >%s<", p_msg);
    }
}
static nrfx_err_t spi_init(){
    nrfx_spim_t spim_inst= NRFX_SPIM_INSTANCE(SPIM_INST_IDX);
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SCK_PIN,
                                                              MOSI_PIN,
                                                              MISO_PIN,
                                                              NRF_SPIM_PIN_NOT_CONNECTED);

    spim_config.frequency = 8000000;
    status = nrfx_spim_init(&spim_inst, &spim_config, NULL, NULL);
    return status;
}

int main(void)
{
    // set clock source and speed
	//init_clock();
    (void)status;

    status = spi_init();

    nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);
    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);    
    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    config.p_context = "Some context";
    status = nrfx_timer_init(&timer_inst, &config, timer_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_timer_clear(&timer_inst);

    // TIME_TO_WAIT_MS sets the period of the timer
    uint32_t desired_ticks = nrfx_timer_ms_to_ticks(&timer_inst, TIME_TO_WAIT_MS);
    NRFX_LOG_INFO("Time to wait: %lu ms", TIME_TO_WAIT_MS);

    nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrfx_timer_enable(&timer_inst);
    NRFX_LOG_INFO("Timer status: %s", nrfx_timer_is_enabled(&timer_inst) ? "enabled" : "disabled");

    while(1) {
        __WFE();
    }
	return 0;
}

void spi_timer_handler(struct k_timer * dummy) {
    //! NRFX_SPIM_XFER_TRX() sets both rx and tx buffer
    //! nrfx_spim_xfer() starts transfer 
    stimming = true;
	// sense ing off
    //! sense_buf_tx needs to be modified somewhere 
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(sense_buf_tx, SENSE_TX_LEN, sense_buf_rx, SENSE_RX_LEN);
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);

    // set the DAC to high
    //! dac_buf_tx is a global that is not modified anywhere
    nrfx_spim_xfer_desc_t spim_xfer_desc1 = NRFX_SPIM_XFER_TRX(dac_buf_tx, DAC_TX_LEN, dac_buf_rx, DAC_RX_LEN);
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc1, 0);

	// DAC off/midline
    //! buffer not changed?
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc1, 0);

	// DAC low
    //! buffer not changed?
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc1, 0);

	// DAC off 
	//! buffer not changed?
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc1, 0);

	// switch short
    nrfx_spim_xfer_desc_t spim_xfer_desc2 = NRFX_SPIM_XFER_TRX(switch_buf_tx, SWITCH_TX_LEN, switch_buf_rx, SWITCH_RX_LEN);
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc2, 0);

	// switch open
    //! change switch_buf_tx to open?
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc2, 0);
    
    // turn sensing back on
    nrfx_spim_xfer_desc_t spim_xfer_desc3 = NRFX_SPIM_XFER_TRX(sense_buf_tx, SENSE_TX_LEN, sense_buf_rx, SENSE_RX_LEN);
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc3, 0);
    stimming = false;
}

/*
static void init_clock() {
	// select the clock source: HFINT (high frequency internal oscillator) or HFXO (external 32 MHz crystal)
	NRF_CLOCK_S->HFCLKSRC = (CLOCK_HFCLKSRC_SRC_HFINT << CLOCK_HFCLKSRC_SRC_Pos);

    // start the clock, and wait to verify that it is running
    NRF_CLOCK_S->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK_S->EVENTS_HFCLKSTARTED == 0);
    NRF_CLOCK_S->EVENTS_HFCLKSTARTED = 0;
}

*/


