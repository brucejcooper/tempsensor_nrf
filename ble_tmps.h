
/** @file
 *
 * @defgroup ble_tmps Temperature Service Server
 * @{
 *
 * @brief Temperature Service Server module.
 *
 * @details This module implements a custom Temperature Service which reports the current temperature.
 *          During initialization, the module adds the Temperature Service and Characteristics
 *          to the BLE stack database.
 *
 *          The application must supply an event handler for receiving Temperature Service
 *          events. Using this handler, the service notifies the application when the
 *          LED value changes.
 *
 *          The service also provides a function for letting the application notify
 *          the state of the Temperature Characteristic to connected peers.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_hids_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_HIDS_BLE_OBSERVER_PRIO,
 *                                   ble_hids_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef BLE_TMPS_H__
#define BLE_TMPS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif


#define BLE_TMPS_BLE_OBSERVER_PRIO 2


/**@brief   Macro for defining a ble_tmps instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_TMPS_DEF(_name)                                                                          \
static ble_tmps_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_TMPS_BLE_OBSERVER_PRIO,                                                     \
                     ble_tmps_on_ble_evt, &_name)

                              
#define TMPS_UUID_BASE        {0x22, 0xa9, 0xc7, 0xe1, 0x84, 0xd2, 0x4a, 0x0a, 0x8e, 0xca, 0x2c, 0x32, 0x00, 0x00, 0x00, 0x00}
#define TMPS_UUID_SERVICE               0x1523
#define TMPS_UUID_TEMP_CHAR             0x1524
#define TMPS_UUID_SAMPLE_PERIOD_CHAR    0x1525


// Forward declaration of the ble_tmps_t type.
typedef struct ble_tmps_s ble_tmps_t;

typedef void (*ble_tmps_sample_period_write_handler_t) (uint16_t conn_handle, ble_tmps_t * p_tmps, uint8_t new_state);

/** @brief LED Temperature Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_tmps_sample_period_write_handler_t sample_period_write_handler; /**< Event handler to be called when the config Characteristic is written. */
} ble_tmps_init_t;

/**@brief LED Temperature Service structure. This structure contains various status information for the service. */
struct ble_tmps_s
{
    uint16_t                    service_handle;      /**< Handle of LED Temperature Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    led_char_handles;    /**< Handles related to the LED Characteristic. */
    ble_gatts_char_handles_t    button_char_handles; /**< Handles related to the Temperature Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the LED Temperature Service. */
    ble_tmps_sample_period_write_handler_t sample_period_write_handler;   /**< Event handler to be called when the config Characteristic is written. */
};


/**@brief Function for initializing the LED Temperature Service.
 *
 * @param[out] p_tmps      LED Temperature Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_tmps_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_tmps_init(ble_tmps_t * p_tmps, const ble_tmps_init_t * p_tmps_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Temperature Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  LED Temperature Service structure.
 */
void ble_tmps_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending a button state notification.
 *
 ' @param[in] conn_handle   Handle of the peripheral connection to which the button state notification will be sent.
 * @param[in] p_tmps         LED Temperature Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_tmps_on_temp_change(uint16_t conn_handle, ble_tmps_t * p_tmps, float temperature);


#ifdef __cplusplus
}
#endif

#endif // BLE_TMPS_H__

/** @} */
