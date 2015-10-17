#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "proprietary_rf.h"
#include "micro_esb.h"
#include "uesb_error_codes.h"
#include "main.h"
#include "syma_218_bkt.h"

#define PROPRIETARY_RF_DEBUG (0x12341234UL)

#define MAX_ADDR_LEN         (5UL)
#define FIXED_PIPE_INDEX     (UESB_ADDRESS_PIPE0)

static void uesb_event_handler(void);

static volatile uesb_payload_t m_tx_payload;
static volatile uesb_config_t  m_uesb_config = UESB_DEFAULT_CONFIG;


/**@brief   Function for handling timeslots in a context with elevated priority.
 */
void QDEC_IRQHandler(void)
{
    // This is called here instead of in the uesb_event_handler for two reasons:
    // 1) Updating the uESB params shouldn't have to lag behind transmissions
    // 2) m_tx_callback should not be run at RADIO_IRQHandler priority
    syma_update((uint8_t*)&m_uesb_config.rf_channel,
		(uint8_t*)&m_uesb_config.rx_address_p0[0],
		(uint8_t*)&m_uesb_config.rf_addr_length,
		(uint8_t*)&m_tx_payload.data[0],
		(uint8_t*)&m_tx_payload.length);

    m_uesb_config.payload_length = m_tx_payload.length;

    if (UESB_SUCCESS != uesb_init((uesb_config_t*)&m_uesb_config))
    {
	app_error_handler(PROPRIETARY_RF_DEBUG,
			  __LINE__,
			  (const uint8_t*)__FILE__);
    }

    if(UESB_SUCCESS != uesb_write_tx_payload((uesb_payload_t*) &m_tx_payload))
    {
	app_error_handler(PROPRIETARY_RF_DEBUG,
			  __LINE__,
			  (const uint8_t*)__FILE__);
    }
}


/**@brief   Function for receiving callbacks from the micro-esb library.
 */
static void uesb_event_handler(void)
{
    // NOTE: This will be executed at the RADIO_IRQHandler priority so execution time
    //       should be kept as short as possible.
    uint32_t rf_interrupts;

    uesb_get_clear_interrupts(&rf_interrupts);

    if(rf_interrupts & UESB_INT_TX_SUCCESS_MSK)
    {
	if (UESB_SUCCESS != uesb_disable())
	{
	    app_error_handler(PROPRIETARY_RF_DEBUG,
			      __LINE__,
			      (const uint8_t*)__FILE__);
	}
	timeslot_finished();
    }

    if(rf_interrupts & UESB_INT_TX_FAILED_MSK)
    {
        uesb_flush_tx();
    }

    if(rf_interrupts & UESB_INT_RX_DR_MSK)
    {
	// The Syma X4 does not transmit.
    }
}


void timeslot_missed(void)
{
    syma_update_missed();
}


void proprietary_rf_init(void)
{
    uint32_t err_code;

    m_uesb_config.rf_channel                     = 64;
    m_uesb_config.crc                            = UESB_CRC_16BIT;
    m_uesb_config.payload_length                 = 10;
    m_uesb_config.protocol                       = UESB_PROTOCOL_SB;
    m_uesb_config.bitrate                        = UESB_BITRATE_250KBPS;
    m_uesb_config.mode                           = UESB_MODE_PTX;
    m_uesb_config.rf_addr_length                 = 5;
    m_uesb_config.tx_output_power                = UESB_TX_POWER_4DBM;
    m_uesb_config.dynamic_ack_enabled            = 0;
    m_uesb_config.dynamic_payload_length_enabled = 0;
    m_uesb_config.rx_pipes_enabled               = 0x01;
    m_uesb_config.retransmit_delay               = 3750;
    m_uesb_config.retransmit_count               = 0;
    m_uesb_config.event_handler                  = uesb_event_handler;
    m_uesb_config.radio_irq_priority             = 0;

    m_tx_payload.pipe = FIXED_PIPE_INDEX;

    // The radio timeslot API will use the QDEC interrupt because that is rarely used.
    err_code = sd_nvic_ClearPendingIRQ(QDEC_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(QDEC_IRQn, NRF_APP_PRIORITY_HIGH);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(QDEC_IRQn);
    APP_ERROR_CHECK(err_code);

    syma_init();
}
