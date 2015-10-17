/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "softdevice_handler.h"
#include "bsp.h"
#include "main.h"
#include "ble_ctl.h"
#include "gamepad_ctl.h"
#include "proprietary_rf.h"

// This is only required on rev 2 silicon.
#define USE_REV_2_SD8_WORKAROUND    (0UL)

// These are used when generating asserts and parsing stack dumps.
#define MAIN_DEBUG                  (0x12345678UL)
#define DEAD_BEEF                   (0xDEADBEEFUL)

#define DBG_PIN_TS_ACTIVE           (12UL)
#define DBG_PIN_TS_MISSED           (13UL)
#define DBG_PINS_ENABLED            (1UL)

#define GAMEPAD_PIN_LATCH           (1UL)
#define GAMEPAD_PIN_CLOCK           (2UL)
#define GAMEPAD_PIN_DATA            (3UL)

// The safety margin is used to ensure that the timeslots are
// finished with time to spare.
#define TS_SAFETY_MARGIN_US         (100UL)
#define TS_PRIORITY                 (NRF_RADIO_PRIORITY_HIGH)

// This is implemented in micro_esb.c
extern void RADIO_IRQHandler(void);

// These are used for timeslot housekeeping.
static bool     m_timeslot_is_active;
static uint32_t m_blocked_cancelled_count;

static nrf_radio_request_t m_timeslot_req = {
    NRF_RADIO_REQ_TYPE_NORMAL,
    .params.normal = {
        NRF_RADIO_HFCLK_CFG_FORCE_XTAL,
	TS_PRIORITY,
	TS_UPDATE_INTERVAL_US,
	TS_LEN_US
    }};

static nrf_radio_signal_callback_return_param_t m_rsc_return = {
    NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,
    .params.request.p_next = &m_timeslot_req
};


static void ts_req_normal_configure(nrf_radio_request_t * req,
				    uint32_t interval_ms,
				    uint32_t blocked_count)
{
    req->request_type              = NRF_RADIO_REQ_TYPE_NORMAL;
    req->params.normal.hfclk       = NRF_RADIO_HFCLK_CFG_FORCE_XTAL;
    req->params.normal.priority    = TS_PRIORITY;
    req->params.normal.length_us   = TS_LEN_US;
    req->params.normal.distance_us = ((blocked_count + 1) * interval_ms);

    if (NRF_RADIO_DISTANCE_MAX_US < req->params.normal.distance_us)
    {
	app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
    }
}


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product.
 *          You need to analyze how your product is supposed to react in case of
 *          error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
		volatile uint32_t error = error_code;
    volatile uint32_t line = line_num;
		volatile uint8_t * file = (uint8_t*)p_file_name;
    // On assert, the system can only recover on reset.
#ifdef DEBUG
    for (;;)
    {
    }
#else
    NVIC_SystemReset();
#endif
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product.
 *          You need to analyse how your product is supposed to react in case
 *          of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief   Function for handling timeslot events.
 */
static nrf_radio_signal_callback_return_param_t * radio_callback (uint8_t signal_type)
{
    // NOTE: This callback runs at lower-stack priority (the highest priority possible).
    switch (signal_type) {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        // TIMER0 is pre-configured for 1Mhz.
        NRF_TIMER0->TASKS_STOP          = 1;
	NRF_TIMER0->TASKS_CLEAR         = 1;
	NRF_TIMER0->MODE                = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
	NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
	NRF_TIMER0->INTENSET            = (TIMER_INTENSET_COMPARE0_Set <<
					   TIMER_INTENSET_COMPARE0_Pos);
	NRF_TIMER0->CC[0]               = (TS_LEN_US - TS_SAFETY_MARGIN_US);
	NRF_TIMER0->BITMODE             = (TIMER_BITMODE_BITMODE_24Bit <<
					   TIMER_BITMODE_BITMODE_Pos);
	NRF_TIMER0->TASKS_START         = 1;

	NRF_RADIO->POWER                = (RADIO_POWER_POWER_Enabled <<
					   RADIO_POWER_POWER_Pos);

	m_blocked_cancelled_count       = 0;
	m_timeslot_is_active            = true;

	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_SetPendingIRQ(QDEC_IRQn);

#ifdef DBG_PINS_ENABLED
	nrf_gpio_pin_set(DBG_PIN_TS_ACTIVE);
#endif
	break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
        if (NRF_TIMER0->EVENTS_COMPARE[0] &&
	    (NRF_TIMER0->INTENSET &
	     (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENCLR_COMPARE0_Pos)))
        {
	    NRF_TIMER0->TASKS_STOP  = 1;
	    NRF_RADIO->POWER        = (RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos);

#ifdef DBG_PINS_ENABLED
	    nrf_gpio_pin_clear(DBG_PIN_TS_ACTIVE);
#endif

	    if (m_timeslot_is_active)
	    {
	        // The timeslot is about to end but processing has not finished.
                app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
	    }
	}

	m_rsc_return.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        ts_req_normal_configure(m_rsc_return.params.request.p_next,
				TS_UPDATE_INTERVAL_US,
				0);
	return &m_rsc_return;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
        // This RADIO_IRQHandler is implemented in the micro-esb library.
        RADIO_IRQHandler();
	break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
    default:
        app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
	break;
    };
    m_rsc_return.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
    return &m_rsc_return;
}


/**@brief   Function for requesting access to the radio
 *
 * @details Requests the earliest-possible timeslot so the radio
 *          can be used when it's not busy with BLE.
 */
static void timeslot_start(void)
{
    uint32_t err_code;
    nrf_radio_request_t timeslot_req_earliest = {
        NRF_RADIO_REQ_TYPE_EARLIEST,
        .params.earliest = {
            NRF_RADIO_HFCLK_CFG_FORCE_XTAL,
 	    NRF_RADIO_PRIORITY_NORMAL,
	    TS_LEN_US,
 	    NRF_RADIO_EARLIEST_TIMEOUT_MAX_US
    }};

    err_code = sd_radio_session_open(radio_callback);
    APP_ERROR_CHECK(err_code);

    err_code = sd_radio_request(&timeslot_req_earliest);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for handling SOC events.
 */
static void soc_evt_handler_callback(uint32_t evt_id)
{
    // NOTE: This callback runs at app-low priority (the second-to-lowest
    //       priority possible).
    uint32_t err_code;

    switch(evt_id) {
    case NRF_EVT_RADIO_BLOCKED:
    case NRF_EVT_RADIO_CANCELED:
	m_blocked_cancelled_count++;
	ts_req_normal_configure(&m_timeslot_req,
				TS_UPDATE_INTERVAL_US,
				m_blocked_cancelled_count);
        err_code = sd_radio_request((nrf_radio_request_t*) &m_timeslot_req);
	APP_ERROR_CHECK(err_code);
	timeslot_missed();

#ifdef DBG_PINS_ENABLED
	nrf_gpio_pin_set(DBG_PIN_TS_MISSED);
	nrf_gpio_pin_clear(DBG_PIN_TS_MISSED);
#endif
	break;
    case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
        app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
        break;
    case NRF_EVT_RADIO_SESSION_CLOSED:
        break;
    case NRF_EVT_RADIO_SESSION_IDLE:
        err_code = sd_radio_session_close();
        APP_ERROR_CHECK(err_code);
	break;
    default:
        break;
  };
}


/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


#if USE_REV_2_SD8_WORKAROUND
/**@brief   Function that allows the SoftDevice to block the CPU while the radio is active.
 */
static void enable_cpu_mutex(void)
{
    uint32_t  err_code;
    ble_opt_t cpu_blocking_disabled;
    uint32_t  opt_id = BLE_COMMON_OPT_RADIO_CPU_MUTEX;

    // The CPU mutex must be enabled for rev 2 silicon.
    cpu_blocking_disabled.common_opt.radio_cpu_mutex.enable = 1;
    err_code = sd_ble_opt_set(opt_id, &cpu_blocking_disabled);
    APP_ERROR_CHECK(err_code);
}
#endif


void timeslot_finished()
{
    m_timeslot_is_active = false;
}


int main(void)
{
    uint32_t err_code;

	bool     ble_connected = false;

    ble_ctl_init();
    gamepad_ctl_init(GAMEPAD_PIN_LATCH, GAMEPAD_PIN_CLOCK, GAMEPAD_PIN_DATA);

#if USE_REV_2_SD8_WORKAROUND
    enable_cpu_mutex();
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_MODE_OFF);
#endif

    err_code = softdevice_sys_evt_handler_set(soc_evt_handler_callback);
    APP_ERROR_CHECK(err_code);

#ifdef DBG_PINS_ENABLED
    nrf_gpio_cfg_output(DBG_PIN_TS_ACTIVE);
    nrf_gpio_cfg_output(DBG_PIN_TS_MISSED);
    nrf_gpio_pin_clear(DBG_PIN_TS_ACTIVE);
    nrf_gpio_pin_clear(DBG_PIN_TS_MISSED);
#endif

    proprietary_rf_init();
    timeslot_start();

    for (;;)
    {
	if (ble_ctl_connected())
	{
	    if (!ble_connected)
	    {
		ble_connected = true;
		if (!gamepad_ctl_disable())
		{
		    app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
		}
	    }
	    power_manage();
	}
	else
	{
	    if (!gamepad_ctl_update())
	    {
		app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
	    }
	}
    }
}
