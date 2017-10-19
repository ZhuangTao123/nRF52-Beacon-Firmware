#include	"sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_BCS)
#include 	"ble_bcs.h"

#include	<stdlib.h>
#include	<string.h>
#include	"app_error.h"
#include	"ble_srv_common.h"

#define BLE_UUID_BCS_ADV_CHARACTERISTIC		0x0001				/**< The UUID of the Advertising Characteristic. */
#define BLE_UUID_BCS_ADVINT_CHARACTERISTIC	0x0002				/**< The UUID of the Advertising Interval Characteristic. */

#define BLE_BCS_MAX_ADV_CHAR_LEN		21
#define BLE_BCS_MAX_ADVINT_CHAR_LEN		2

#define BCS_BASE_UUID					{{0x95, 0xE2, 0xED, 0xEB, 0x1B, 0xA0, 0x39, 0x8A, 0xDF, 0x4B, 0xD3, 0x8E, 0x00, 0x00, 0xC8, 0xA3}} /**< Used vendor specific UUID. */


/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
*
* @param[in] p_bcs     Beacon Service structure.
* @param[in] p_ble_evt Pointer to the event received from BLE stack.
*/
static void on_connect(ble_bcs_t * p_bcs, ble_evt_t const * p_ble_evt)
{
	p_bcs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the SoftDevice.
*
* @param[in] p_bcs     Beacon Service structure.
* @param[in] p_ble_evt Pointer to the event received from BLE stack.
*/
static void on_disconnect(ble_bcs_t * p_bcs, ble_evt_t const * p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_bcs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
*
* @param[in] p_bcs     Beacon Service structure.
* @param[in] p_ble_evt Pointer to the event received from BLE stack.
*/
static void on_write(ble_bcs_t * p_bcs, ble_evt_t const * p_ble_evt)
{
	ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	ble_bcs_evt_t evt;
	evt.p_bcs = p_bcs;
	
	if((p_evt_write->handle == p_bcs->advertising_handle.value_handle)
	   && (p_bcs->data_handler != NULL))
	{
		evt.params.rx_data.p_data = p_evt_write->data;
		evt.params.rx_data.length = p_evt_write->len;
		evt.type = BLE_BCS_EVT_ADV_RX_DATA;
		p_bcs->data_handler(&evt);
	}
	else if((p_evt_write->handle == p_bcs->advint_handle.value_handle)
			&& (p_bcs->data_handler != NULL))
	{
		evt.params.rx_data.p_data = p_evt_write->data;
		evt.params.rx_data.length = p_evt_write->len;
		evt.type = BLE_BCS_EVT_ADVINT_RX_DATA;
		p_bcs->data_handler(&evt);
	}
	else
	{
		// Do Nothing. This event is not relevant for this service.
	}
}

/**@brief Function for adding advertising characteristic.
*
* @param[in] p_bcs       Beacon Service structure.
* @param[in] p_bcs_init  Information needed to initialize the service.
*
* @return NRF_SUCCESS on success, otherwise an error code.
*/
static uint32_t advertising_char_add(ble_bcs_t * p_bcs, const ble_bcs_init_t * p_bcs_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	memset(&char_md, 0, sizeof(char_md));
	
	char_md.char_props.read			 = 1;
	char_md.char_props.write         = 1;
	char_md.char_props.write_wo_resp = 0;
	char_md.p_char_user_desc         = NULL;
	char_md.p_char_pf                = NULL;
	char_md.p_user_desc_md           = NULL;
	char_md.p_cccd_md                = NULL;
	char_md.p_sccd_md                = NULL;
	
	ble_uuid.type = p_bcs->uuid_type;
	ble_uuid.uuid = BLE_UUID_BCS_ADV_CHARACTERISTIC;
	
	memset(&attr_md, 0, sizeof(attr_md));
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
	attr_md.vloc    = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen    = 1;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	
	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = BLE_BCS_MAX_ADV_CHAR_LEN;
	attr_char_value.p_value	  = NULL;
	
	return sd_ble_gatts_characteristic_add(p_bcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_bcs->advertising_handle);
}

/**@brief Function for adding advertising interval characteristic.
*
* @param[in] p_bcs       Beacon Service structure.
* @param[in] p_bcs_init  Information needed to initialize the service.
*
* @return NRF_SUCCESS on success, otherwise an error code.
*/
static uint32_t advint_char_add(ble_bcs_t * p_bcs, const ble_bcs_init_t * p_bcs_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	memset(&char_md, 0, sizeof(char_md));
	
	char_md.char_props.read			 = 1;
	char_md.char_props.write         = 1;
	char_md.char_props.write_wo_resp = 1;
	char_md.p_char_user_desc         = NULL;
	char_md.p_char_pf                = NULL;
	char_md.p_user_desc_md           = NULL;
	char_md.p_cccd_md                = NULL;
	char_md.p_sccd_md                = NULL;
	
	ble_uuid.type = p_bcs->uuid_type;
	ble_uuid.uuid = BLE_UUID_BCS_ADVINT_CHARACTERISTIC;
	
	memset(&attr_md, 0, sizeof(attr_md));
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
	attr_md.vloc    = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen    = 0;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	
	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = sizeof(uint16_t);
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = sizeof(uint16_t);
	attr_char_value.p_value   = (uint8_t *)&p_bcs_init->init_advint_value;
	
	return sd_ble_gatts_characteristic_add(p_bcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_bcs->advint_handle);
}


void ble_bcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
	if ((p_context == NULL) || (p_ble_evt == NULL))
	{
		return;
	}
	
	ble_bcs_t * p_bcs = (ble_bcs_t *)p_context;
	
	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		on_connect(p_bcs, p_ble_evt);
		break;
		
	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(p_bcs, p_ble_evt);
		break;
		
	case BLE_GATTS_EVT_WRITE:
		on_write(p_bcs, p_ble_evt);
		break;
		
	default:
		// No implementation needed.
		break;
	}
}

uint32_t ble_bcs_init(ble_bcs_t * p_bcs, ble_bcs_init_t const * p_bcs_init)
{
	uint32_t      err_code;
	ble_uuid_t    ble_uuid;
	ble_uuid128_t bcs_base_uuid = BCS_BASE_UUID;
	
	VERIFY_PARAM_NOT_NULL(p_bcs);
	VERIFY_PARAM_NOT_NULL(p_bcs_init);
	
	// Initialize the service structure.
	p_bcs->conn_handle             = BLE_CONN_HANDLE_INVALID;
	p_bcs->data_handler            = p_bcs_init->data_handler;
	p_bcs->is_notification_enabled = false;
	
	/**@snippet [Adding proprietary Service to the SoftDevice] */
	// Add a custom base UUID.
	err_code = sd_ble_uuid_vs_add(&bcs_base_uuid, &p_bcs->uuid_type);
	VERIFY_SUCCESS(err_code);
	
	ble_uuid.type = p_bcs->uuid_type;
	ble_uuid.uuid = BLE_UUID_BCS_SERVICE;
	
	// Add the service.
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
										&ble_uuid,
										&p_bcs->service_handle);
	/**@snippet [Adding proprietary Service to the SoftDevice] */
	VERIFY_SUCCESS(err_code);
	
	// Add the Advertising Characteristic.
	err_code = advertising_char_add(p_bcs, p_bcs_init);
	VERIFY_SUCCESS(err_code);
	
	// Add the Advertising Interval Characteristic.
	err_code = advint_char_add(p_bcs, p_bcs_init);
	VERIFY_SUCCESS(err_code);
	
	return NRF_SUCCESS;
}

uint32_t ble_bcs_advertising_interval_set(ble_bcs_t * p_bcs, uint16_t new_interval)
{
	ble_gatts_value_t gatts_value;
	
	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));
	
	gatts_value.len     = sizeof(uint16_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = (uint8_t*)&new_interval;
	
	// Update database
	return sd_ble_gatts_value_set(p_bcs->conn_handle,
								  p_bcs->advint_handle.value_handle,
								  &gatts_value);
}

#endif

