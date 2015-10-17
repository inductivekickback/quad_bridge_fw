#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "ble_ctl.h"
#include "syma_218_bkt.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT (0UL)

#define DEVICE_NAME                     ("SYMA_218")

#define APP_ADV_INTERVAL                (1600UL) // Adv. interval (in units of 0.625ms)
#define APP_ADV_TIMEOUT_IN_SECONDS      (0UL)  // Adv. timeout (in units of s)

#define APP_TIMER_PRESCALER             (0UL)  // Value of the RTC1 PRESCALER register
#define APP_TIMER_MAX_TIMERS            (2 + BSP_APP_TIMERS_NUMBER)
#define APP_TIMER_OP_QUEUE_SIZE         (4UL)

#define APP_GPIOTE_MAX_USERS            (1UL)

#define CONN_SUP_TIMEOUT                (MSEC_TO_UNITS(4000, UNIT_10_MS))
#define FIRST_CONN_PARAMS_UPDATE_DELAY  (APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER))
#define NEXT_CONN_PARAMS_UPDATE_DELAY   (APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER))
#define MAX_CONN_PARAMS_UPDATE_COUNT    (3UL)

#define SEC_PARAM_BOND                  (0UL)
#define SEC_PARAM_MITM                  (0UL)
#define SEC_PARAM_IO_CAPABILITIES       (BLE_GAP_IO_CAPS_NONE)
#define SEC_PARAM_OOB                   (0UL)
#define SEC_PARAM_MIN_KEY_SIZE          (7UL)
#define SEC_PARAM_MAX_KEY_SIZE          (16UL)

#define UART_TX_BUF_SIZE                (256UL)
#define UART_RX_BUF_SIZE                (256UL)

#define DBG_PIN_BLE_WRITE               (14UL)
#define DBG_PINS_ENABLED                (1UL)

static ble_gap_sec_params_t m_sec_params;
static ble_nus_t            m_nus;
static uint16_t             m_conn_handle = BLE_CONN_HANDLE_INVALID;

static uint8_t              m_response_buf[BLE_NUS_MAX_DATA_LEN];

typedef enum
{
    BLE_CMD_BIND = 0,
    BLE_CMD_CTL,
    BLE_CMD_UNBIND,
    BLE_CMD_COUNT
} ble_cmd_t;

typedef enum
{
    BLE_RESPONSE_BOUND = 0,
    BLE_RESPONSE_ERROR,
    BLE_RESPONSE_UNBOUND,
    BLE_RESPONSE_COUNT
} ble_response_t;

#define BLE_RESPONSE_LEN                (1UL)

static const uint8_t BLE_CMD_LEN[BLE_CMD_COUNT] = {1, 5, 1};

/**@brief   Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access
 *          Profile) parameters of the device. It also sets the permissions
 *          and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting the advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordi
 *           UART BLE Service and send it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be processed.
 * @param[in] length   Length of the data.
 */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    uint32_t err_code;

    if (0 == length)
    {
	return;
    }

#ifdef DBG_PINS_ENABLED
    nrf_gpio_pin_set(DBG_PIN_BLE_WRITE);
    nrf_gpio_pin_clear(DBG_PIN_BLE_WRITE);
#endif

    if ((BLE_CMD_COUNT <= p_data[0]) || (BLE_CMD_LEN[p_data[0]] != length))
    {
	m_response_buf[0] = BLE_RESPONSE_ERROR;
	err_code = ble_nus_string_send(p_nus,
				       &m_response_buf[0],
				       BLE_RESPONSE_LEN);
	APP_ERROR_CHECK(err_code);
	return;
    }

    switch (p_data[0])
    {
    case BLE_CMD_BIND:
	if (!syma_is_bound())
	{
	    syma_bind();
	}

	m_response_buf[0] = BLE_RESPONSE_BOUND;
	err_code = ble_nus_string_send(p_nus,
				       &m_response_buf[0],
				       BLE_RESPONSE_LEN);
	APP_ERROR_CHECK(err_code);
	break;
    case BLE_CMD_CTL:
	syma_throttle_set(p_data[1]);
	syma_pitch_set(p_data[2]);
	syma_roll_set(p_data[3]);
	syma_yaw_set(p_data[4]);
	break;
    case BLE_CMD_UNBIND:
	syma_throttle_set(SYMA_MIN_THROTTLE_VALUE);

	m_response_buf[0] = BLE_RESPONSE_UNBOUND;
	err_code = ble_nus_string_send(p_nus,
				       &m_response_buf[0],
				       BLE_RESPONSE_LEN);

	// NOTE: It's OK if the central disconnects before this
	//       notification is received.
	if (NRF_ERROR_INVALID_STATE != err_code)
	{
	    APP_ERROR_CHECK(err_code);
	}
	break;
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t   nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection
 *              Parameters Module which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done
 *              by simply setting the disconnect_on_fail config parameter, but
 *              instead we use the event handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}

/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                    err_code;
    static ble_gap_sec_keyset_t s_sec_keyset;
    ble_gap_enc_info_t *        p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
	    syma_init();
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();

	    if (syma_is_bound())
	    {
		syma_throttle_set(0);
	    }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            s_sec_keyset.keys_periph.p_enc_key = NULL;
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params,
                                                   &s_sec_keyset);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            if (s_sec_keyset.keys_periph.p_enc_key != NULL)
            {
                p_enc_info = &s_sec_keyset.keys_periph.p_enc_key->enc_info;
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device.
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules
 *        with a S110 SoftDevice event handler.
 *
 * @details This function is called from the S110 SoftDevice event
 *          interrupt handler after a S110 SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


bool ble_ctl_connected(void)
{
    return (BLE_CONN_HANDLE_INVALID != m_conn_handle);
}


void ble_ctl_init(void)
{
    uint32_t err_code;

    APP_TIMER_INIT(APP_TIMER_PRESCALER,
		   APP_TIMER_MAX_TIMERS,
		   APP_TIMER_OP_QUEUE_SIZE,
		   false);
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);

    ble_stack_init();

    err_code = bsp_init((BSP_INIT_LED | BSP_INIT_BUTTONS),
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        NULL);
    APP_ERROR_CHECK(err_code);
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();

    advertising_start();

#ifdef DBG_PINS_ENABLED
    nrf_gpio_cfg_output(DBG_PIN_BLE_WRITE);
    nrf_gpio_pin_clear(DBG_PIN_BLE_WRITE);
#endif
}
