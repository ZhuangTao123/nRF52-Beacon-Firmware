#ifndef	BLE_BCS_H__
#define	BLE_BCS_H__

#include	<stdint.h>
#include	"ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_bcs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_BCS_DEF(_name)																			\
static ble_bcs_t	_name;																			\
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_BCS_BLE_OBSERVER_PRIO,                                                     \
                     ble_bcs_on_ble_evt, &_name)


#define BLE_UUID_BCS_SERVICE 0x6500                      /**< The UUID of the Beacon Service. */

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_BCS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE)
#else
    #define BLE_BCS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief   Beacon Service event types. */
typedef enum
{
    BLE_BCS_EVT_ADV_RX_DATA,       	/**< Advertising data received. */
	BLE_BCS_EVT_ADVINT_RX_DATA,		/**< Advertising interval data received. */
    BLE_BCS_EVT_COMM_STARTED,      	/**< Notification has been enabled. */
    BLE_BCS_EVT_COMM_STOPPED,      	/**< Notification has been disabled. */
} ble_bcs_evt_type_t;


/* Forward declaration of the ble_bcs_t type. */
typedef struct ble_bcs_s ble_bcs_t;


/**@brief   Beacon Service @ref BLE_BCS_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCS_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data;           /**< A pointer to the buffer with received data. */
    uint16_t        length;           /**< Length of received data. */
} ble_bcs_evt_rx_data_t;


/**@brief   Beacon Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
	ble_bcs_evt_type_t	type;
	ble_bcs_t	* p_bcs;
	union
	{
		ble_bcs_evt_rx_data_t	rx_data;
	} params;
} ble_bcs_evt_t;

/**@brief   Beacon Service event handler type. */
typedef void (*ble_bcs_data_handler_t) (ble_bcs_evt_t * p_evt);

/**@brief   Beacon Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_bcs_init
 *          function.
 */
typedef struct
{
	uint16_t	init_advint_value;		 /**< Initial advertising interval passed to the service. */
    ble_bcs_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_bcs_init_t;

/**@brief   Nordic Beacon Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_bcs_s
{
	uint8_t						uuid_type;					/**< UUID type for Beacon Service Base UUID. */
	uint16_t					service_handle;				/**< Handle of Beacon Service (as provided by the SoftDevice). */
	ble_gatts_char_handles_t	advertising_handle;			/**< Handles related to the advertising characteristic (as provided by the SoftDevice). */
	ble_gatts_char_handles_t	advint_handle;				/**< Handles related to the advertising interval characteristic (as provided by the SoftDevice). */
	uint16_t					conn_handle;				/**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
	bool						is_notification_enabled;	/**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
	ble_bcs_data_handler_t		data_handler;				/**< Event handler to be called for handling received data. */
};

/**@brief	Function for initializing the Beacon Service.
 *
 * @param[out] p_bcs	  Beacon Service structure. This structure must be supplied
 *						  by the application. It is initialized by this function and will
 *						  later be used to identify this particular service instance.
 * @param[in] p_bcs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_bcs or p_bcs_init is NULL.
 */
uint32_t ble_bcs_init(ble_bcs_t * p_bcs, ble_bcs_init_t const * p_bcs_init);


/**@brief	Function for handling the Beacon Service's BLE events.
 *
 * @details The Beacon Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Beacon Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt 	Event received from the SoftDevice.
 * @param[in] p_context 	Beacon Service structure.
 */
void ble_bcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for setting the advertising interval.
 *
 * @param[in]   p_bcs            Beacon Service structure.
 * @param[in]   advint   		 New advertising interval (unit ms).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_bcs_advertising_interval_set(ble_bcs_t * p_bcs, uint16_t new_interval);

#ifdef __cplusplus
}
#endif

#endif // BLE_BCS_H__

/** @} */

