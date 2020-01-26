/**
 * @brief Temperature sensor Application main file.
 *
 * This file contains the source code for a sample server application using the Temperature service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "ble_tmps.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "inttypes.h"
#include "device_config.h"
#include "fds.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "tmp11x.h"


#define CONFIG_FILE     (0x7E10)
#define CONFIG_REC_KEY  (0x7E20)

#define DEFAULT_DEVICE_NAME             "New Temp Sensor"
#define MANUFACTURER_NAME               "CCPEED"                                /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "TMP002"                                /**< Model number. Will be passed to Device Information Service. */
#define BUILD_NUM                       "1"
#define HW_REV                          "1"
#define MANUFACTURER_ID                 0x1122334455                            /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x667788                                /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define LINK_TOTAL                      NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                                        NRF_SDH_BLE_CENTRAL_LINK_COUNT


#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(10000)                       /**< Battery level measurement interval (ticks). */


#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the Temperature Service. */
#define TIMER_LED                       BSP_BOARD_LED_3


#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

/**
 * iOS has guidelines about BLE connection parameters (from https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf)
 * The accessory should first use the recommended advertising interval of 20 ms for at least 30 seconds. If it is not discovered 
 * within the initial 30 seconds, Apple recommends using one of the following longerintervals to increase chances of discovery 
 * by the device:
 * 
 * * 152.5 ms
 * * 211.25 ms
 * * 318.75 ms
 * * 417.5 ms
 * * 546.25 ms
 * * 760 ms
 * * 852.5 ms
 * * 1022.5 ms
 * * 1285 ms 
 * We want to maximise our battery life, and discovery isnt' that important, so we'll use the maximum value.
 */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(1285, UNIT_0_625_MS)      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 2 seconds). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

/**
 * Also from the iOS guidelines.
 * The connection parameter request may be rejected if it does not comply with all of these rules:
 * * Slave Latency ≤30
 * * 2 seconds ≤ connSupervisionTimeout ≤ 6 seconds
 * * Interval Min modulo 15 ms == 0
 * * Interval Min ≥15 ms
 * * One of the following:
 *     * Interval Min + 15 ms ≤ Interval Max
 *     * Interval Min == Interval Max == 15 ms
 * * Interval Max * (Slave Latency + 1) ≤ 2 seconds
 * * Interval Max * (Slave Latency + 1) * 3 < connSupervisionTimeout
 * 
 * Make sure all the values here comply. The last one in particular is important if large connection intervals are allowed.
 */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(105, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(330, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (2 second). */
#define SLAVE_LATENCY                   5                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)         /**< Connection supervisory time-out (10 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define TIMER_DURATION                  APP_TIMER_TICKS(1000)                   /**< How often the timer to fetch temperature goes off, when connected. */

BLE_TMPS_DEF(m_tmps);                                                           /**< Temperature Service instance. */
BLE_BAS_DEF(m_bas);                                                             /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                          /**< Context for the Queued Write module.*/

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */


static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};


ccpeed_cfg_t ccpeed_cfg = {
    .device_name = DEFAULT_DEVICE_NAME,
    .sampling_period = 1,
};



static bool volatile m_fds_initialized;



/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};



/**< Handler for repeated timer used to blink LED 1. */
APP_TIMER_DEF(m_connected_timer_id);     
APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */



static void connected_timer_handler(void * p_context)
{
    bsp_board_led_invert(TIMER_LED);

    // Request the value of the current register (should be reg 0, which is the temperature). 
    // The result will be deliverd to the callback. 
    tmp11x_initiate_read();

    // nrf_drv_twi_disable(&twi);

}


static void temperature_callback(float tmp) {
    // NRF_LOG_INFO("Current Temperature is %d", (int) (tmp*100));
    static float lastTmp = -1E10;

    // Only send a notify if the temperature has actually changed (which it probably will have)
    if (tmp != lastTmp) {
        ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();

        for (uint8_t i = 0; i < conn_handles.len; i++)
        {
            ret_code_t err_code = ble_tmps_on_temp_change(conn_handles.conn_handles[i], &m_tmps, tmp);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
        lastTmp = tmp;
    }
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}



/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)ccpeed_cfg.device_name, strlen(ccpeed_cfg.device_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{TMPS_UUID_SERVICE, m_tmps.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t) 95; // TODO actually get a real value from the board

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers
    err_code = app_timer_create(&m_connected_timer_id, APP_TIMER_MODE_REPEATED, connected_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

}



const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}


static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->result == NRF_SUCCESS) {
        NRF_LOG_INFO("Event: %s received (NRF_SUCCESS)", fds_evt_str[p_evt->id]);
    } else {
        NRF_LOG_INFO("Event: %s received (%s)", fds_evt_str[p_evt->id], fds_err_str(p_evt->result));
    }

    switch (p_evt->id) {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS) {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
            break;

        case FDS_EVT_DEL_RECORD:
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            // m_delete_all.pending = false;
            break;

        default:
            break;
    }
}

void persist_config() {
    fds_record_t        record;
    fds_record_desc_t desc = {0};

    record.file_id = CONFIG_FILE;
    record.key = CONFIG_REC_KEY;
    record.data.p_data = &ccpeed_cfg;
    record.data.length_words = (sizeof(ccpeed_cfg_t) + 3) / 4;

    NRF_LOG_INFO("Writing %s", (char *) record.data.p_data);
    ret_code_t err_code = fds_record_write(&desc, &record);
    APP_ERROR_CHECK(err_code);
}

void read_config() {
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};
    fds_flash_record_t config = {0};
    bool found = false;


    // Data is stored as a linked list of entries.  We need the last one that matches our File and Key combo.
    while ((err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok)) == NRF_SUCCESS) {
        /* Open the record and read its contents. */
        err_code = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(err_code);

        /* Copy the configuration from flash into ccpeed_cfg. */
        memcpy(&ccpeed_cfg, config.p_data, sizeof(ccpeed_cfg_t));
        /* Close the record when done reading. */
        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
        found = true;
    } 
    
    if (found) {
        NRF_LOG_INFO("Config file found - Value is %s (record id %d)", (char *) config.p_data, desc.record_id);
    } else {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing config file with defaults");
        persist_config();
    }
}

/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_tmps     Instance of Temperature Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void sample_period_write_handler(uint16_t conn_handle, ble_tmps_t * p_tmps, uint8_t sample_period)
{
    NRF_LOG_INFO("Sample period set to %d", sample_period);
    ccpeed_cfg.sampling_period = sample_period;
    persist_config();
}


static void name_write_handler(uint16_t conn_handle, ble_tmps_t * p_tmps, const char * new_name, size_t len)
{
    memcpy(ccpeed_cfg.device_name, new_name, len);
    ccpeed_cfg.device_name[len] = 0;

    NRF_LOG_INFO("Device Name set to %s ", ccpeed_cfg.device_name);

    // Because we have changed the device name, reset the GAP parameters.
    gap_params_init();
    persist_config();
}



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_tmps_init_t    tmps_init = {0};
    ble_bas_init_t     bas_init = {0};
    ble_dis_init_t     dis_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;


    for (uint32_t i = 0; i < LINK_TOTAL; i++) {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }


    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, BUILD_NUM);

    char hw_rev[128];
    uint8_t *p = (uint8_t *) &(NRF_FICR->INFO.VARIANT);
    snprintf(hw_rev, sizeof(hw_rev), "nRF%lx Var: %c%c%c%c, RAM: %ldKB, FLASH: %ldKB", NRF_FICR->INFO.PART, p[3], p[2], p[1], p[0], NRF_FICR->INFO.RAM, NRF_FICR->INFO.FLASH);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, hw_rev);


    char serial_num[128];
    snprintf(serial_num, sizeof(serial_num), "%08" PRIx32 "%08" PRIx32, NRF_FICR->DEVICEID[0], NRF_FICR->DEVICEID[1]);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, serial_num);
    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize TMPS.
    tmps_init.sample_period_write_handler = sample_period_write_handler;
    tmps_init.name_write_handler = name_write_handler;
    tmps_init.name_init = ccpeed_cfg.device_name;
    tmps_init.sample_period_init = ccpeed_cfg.sampling_period;
    err_code = ble_tmps_init(&m_tmps, &tmps_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
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
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_on(ADVERTISING_LED);
}




/**@brief Function for handling the Connected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_connected(const ble_gap_evt_t * const p_gap_evt)
{
    ret_code_t  err_code;
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection with link 0x%x established.", p_gap_evt->conn_handle);

    // Assign connection handle to available instance of QWR module.
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }

    err_code = app_timer_start(m_connected_timer_id, TIMER_DURATION, NULL);
    APP_ERROR_CHECK(err_code);

    // Update LEDs
    bsp_board_led_on(CONNECTED_LED);
    if (periph_link_cnt == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
    {
        bsp_board_led_off(ADVERTISING_LED);
    }
    else
    {
        // Continue advertising. More connections can be established because the maximum link count has not been reached.
        advertising_start();
    }
}


/**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_disconnected(ble_gap_evt_t const * const p_gap_evt)
{
    ret_code_t  err_code;
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                 p_gap_evt->conn_handle,
                 p_gap_evt->params.disconnected.reason);

    if (periph_link_cnt == 0) {
        bsp_board_led_off(CONNECTED_LED);

        // Stop the timer.
        bsp_board_led_off(TIMER_LED);
        err_code = app_timer_stop(m_connected_timer_id);
        APP_ERROR_CHECK(err_code);

    }

    if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1)) {
        // Advertising is not running when all connections are taken, and must therefore be started.
        advertising_start();
    }
}




/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connected(&p_ble_evt->evt.gap_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(&p_ble_evt->evt.gap_evt);
            
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}



static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/** @brief Wait for fds to initialize. */
static void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
        idle_state_handle();
    }
}


/**@brief Function for application main entry.
 */
int main(void) {
    // Initialize.
    log_init();
    power_management_init();


    (void) fds_register(fds_evt_handler);
    ret_code_t err_code =  fds_init();
    APP_ERROR_CHECK(err_code);

    wait_for_fds_ready();


    read_config();
    NRF_LOG_INFO("device name is %s. Sampling period is %d", ccpeed_cfg.device_name, ccpeed_cfg.sampling_period);


    leds_init();
    timers_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    tmp11x_init(temperature_callback);


    application_timers_start();

    // Start execution.
    NRF_LOG_INFO("Temp sensor started.");
    advertising_start();


    // Enter main loop.
    for (;;) {
        idle_state_handle();
    }
}


/**
 * @}
 */
