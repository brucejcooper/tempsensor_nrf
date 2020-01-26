
#include "tmp11x.h"
#include "nrf_drv_twi.h"
#include "nrf_twim.h"
#include "nrf_gpio.h"
#include "nrf_log.h"


#define TMP_CONVERSION_CODE 0.0078125

static const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(0);
static const nrf_drv_twi_config_t twi_config = {
    .scl = NRF_GPIO_PIN_MAP(0,27),
    .sda = NRF_GPIO_PIN_MAP(0,26),
    .frequency = NRF_DRV_TWI_FREQ_400K, 
    .interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .clear_bus_init     = TWI_DEFAULT_CONFIG_CLR_BUS_INIT,
    .hold_bus_uninit    = TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT,
};
static uint8_t tmpreg_rx[2];
static uint8_t tx_buf[3];
static tmp11x_callback_t tmp_callback = NULL;




static void set_tmp11x_cfg(uint16_t config) {
    // Set the config
    tx_buf[0] = 0x01;         // Config register
    tx_buf[1] = config >> 8;   // High byte - Specifies one-shot conversion and 1 second Conversion time
    tx_buf[2] = config & 0xFF; // Low byte - 1 sample averaging
    NRF_LOG_ERROR("Setting TMP11X config to 0x%02x%02x", tx_buf[1], tx_buf[2])

    ret_code_t err_code = nrf_drv_twi_tx(&twi, TMP11X_ADDRESS, tx_buf, 3, false);
    APP_ERROR_CHECK(err_code);
}

static void set_tmp11x_register(uint8_t reg) {
    // Set the pointer register to 0x00 (temperature).
    tx_buf[0] = reg;       // Temperature register
    ret_code_t err_code = nrf_drv_twi_tx(&twi, TMP11X_ADDRESS, tx_buf, 1, false);
    APP_ERROR_CHECK(err_code);
}


static void twi_callback(nrf_drv_twi_evt_t const * p_event, void *p_context) {
    if (p_event->type != NRF_DRV_TWI_EVT_DONE) {
        NRF_LOG_ERROR("TWI Event did not complete correctly");
        return;
    }

    ASSERT(p_event->xfer_desc.primary_length >= 1);
    uint8_t reg = p_event->xfer_desc.p_primary_buf[0];

    if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) {
        ASSERT(reg == 0);
        uint16_t code = (p_event->xfer_desc.p_primary_buf[0] << 8) | p_event->xfer_desc.p_primary_buf[1];
        float tmp = code * TMP_CONVERSION_CODE;
        if (tmp_callback == NULL) {
            NRF_LOG_ERROR("No temperature callback");
        } else {
            tmp_callback(tmp);
        }
    } else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX) {
        ASSERT(p_event->xfer_desc.primary_length == 3);
        switch (reg) {
            case 0x01:
                NRF_LOG_INFO("Configuration set.  Resetting register to temp");
                set_tmp11x_register(0x00);
                break;
            default:
                NRF_LOG_WARNING("Callback for write to %d", reg);
                break;
        }
    } else {
        NRF_LOG_WARNING("Unexpected TWI callback");
    }
}



void tmp11x_initiate_read() {
    ret_code_t err_code = nrf_drv_twi_rx(&twi, TMP11X_ADDRESS, tmpreg_rx, sizeof(tmpreg_rx));
    APP_ERROR_CHECK(err_code);
}


void tmp11x_init(tmp11x_callback_t callback) {
    tmp_callback = callback;
    // We only enable/init the TWI interface on demand, otherwise it will consume
    ret_code_t err_code = nrf_drv_twi_init(&twi, &twi_config, twi_callback, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&twi);

    set_tmp11x_cfg(TMP11X_CONFIG_CONV_1s | TMP11X_CONFIG_MODE_CONTINUOUS | TMP11X_CONFIG_AVG1);
}

