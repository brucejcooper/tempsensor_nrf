#include "sdk_common.h"
#include "ble_tmps.h"
#include "ble_srv_common.h"
#include "nrf_log.h"




/**@brief Function for handling the Write event.
 *
 * @param[in] p_tmps      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_tmps_t * p_tmps, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (   (p_evt_write->handle == p_tmps->led_char_handles.value_handle)
        && (p_evt_write->len == 1)
        && (p_tmps->sample_period_write_handler != NULL))
    {
        p_tmps->sample_period_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_tmps, p_evt_write->data[0]);
    }
}


void ble_tmps_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_tmps_t * p_tmps = (ble_tmps_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_tmps, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_tmps_init(ble_tmps_t * p_tmps, const ble_tmps_init_t * p_tmps_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;

    // Initialize service structure.
    p_tmps->sample_period_write_handler = p_tmps_init->sample_period_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {TMPS_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_tmps->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_tmps->uuid_type;
    ble_uuid.uuid = TMPS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_tmps->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add Temperature characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = TMPS_UUID_TEMP_CHAR;
    add_char_params.uuid_type         = p_tmps->uuid_type;
    add_char_params.init_len          = sizeof(uint16_t);
    add_char_params.max_len           = sizeof(uint16_t);
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_tmps->service_handle,
                                  &add_char_params,
                                  &p_tmps->button_char_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add sample period characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = TMPS_UUID_SAMPLE_PERIOD_CHAR;
    add_char_params.uuid_type        = p_tmps->uuid_type;
    add_char_params.init_len         = sizeof(uint8_t);
    add_char_params.max_len          = sizeof(uint8_t);
    add_char_params.char_props.read  = 1;
    add_char_params.char_props.write = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    return characteristic_add(p_tmps->service_handle, &add_char_params, &p_tmps->led_char_handles);
}


uint32_t ble_tmps_on_temp_change(uint16_t conn_handle, ble_tmps_t * p_tmps, float temperature)
{
    int16_t decimalTmp = temperature * 100;
    ble_gatts_hvx_params_t params;
    uint8_t encoded[2];
    uint16_t len = sizeof(encoded);
    encoded[0] = decimalTmp >> 8;
    encoded[1] = decimalTmp & 0xFF;

    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_tmps->button_char_handles.value_handle;
    params.p_data = encoded;
    params.p_len  = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}