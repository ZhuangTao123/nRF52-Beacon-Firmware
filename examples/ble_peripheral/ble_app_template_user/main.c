/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"

// SAADC include files
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// DFU include files
#include "nrf_dfu_svci.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"

// Custom include files
#include "ble_bas.h"
#include "ble_bcs.h"
#include "ble_tps.h"
#include "bdef_file_m.h"


#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "LBTest"                       			/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_FAST_INTERVAL           320                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 100 ms). */
#define APP_ADV_FAST_TIMEOUT            0                                       /**< The advertising timeout in units of seconds. */
#define APP_ADV_SLOW_INTERVAL           1600                                    /**< The advertising interval (in units of 0.625 ms. This value corresponds to 1000 ms). */
#define APP_ADV_SLOW_TIMEOUT            0                                       /**< The advertising timeout in units of seconds. */
#define LBATALERT_INTERVAL				100
#define LBATALERT_TIMEOUT_INTERVAL		APP_TIMER_TICKS(2000)

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  1                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_DISPLAY_ONLY            /**< Display I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define APP_BEACON_INFO_LENGTH          0x17                                    /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                                    /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                                    /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                                    /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x004C                                  /**< Company identifier for Apple Inc. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x00, 0x01                              /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x00, 0x30                              /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x67, 0xd3, 0xa4, 0x02, \
                                        0x23, 0x23, 0x45, 0x74, \
                                        0x93, 0x42, 0x7b, 0x94, \
                                        0x7d, 0xc4, 0x30, 0xd1                  /**< Proprietary UUID for Beacon. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                      /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                              /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

#define TX_POWER_LEVEL					-4
#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define PASSKEY_TXT                     "Passkey:"                              /**< Message to be displayed together with the pass-key. */
#define PASSKEY_TXT_LENGTH              8                                       /**< Length of message to be displayed together with the pass-key. */
#define PASSKEY_LENGTH                  6                                       /**< Length of pass-key received by the stack for display. */
#define SECURITY_REQUEST_DELAY          APP_TIMER_TICKS(400)                    /**< Delay after connection until Security Request is sent, if necessary (ticks). */

#define BTN_ID_LEDS_SWITCH				2
#define BTN_ACTION_LEDS_SWITCH_ON		BSP_BUTTON_ACTION_PUSH
#define BTN_ACTION_LEDS_SWITCH_OFF		BSP_BUTTON_ACTION_LONG_PUSH
#define BTN_ID_LBAT_TEST_SWITCH			1
#define BTN_ACTION_LBAT_TRIGGER_ON		BSP_BUTTON_ACTION_PUSH


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
BLE_BAS_DEF(m_bas);                                            			        /**< Structure used to identify the battery service. */
BLE_BCS_DEF(m_bcs);																/**< Beacon Service instance. */
BLE_TPS_DEF(m_tps);																/**< Tx Power Service instance. */
APP_TIMER_DEF(m_sec_req_timer_id);                                              /**< Security Request timer. */
APP_TIMER_DEF(m_saadc_timer_id);												/**< SAADC timer. */
APP_TIMER_DEF(m_lbat_timer_id);



// Global variables in the application
static pm_peer_id_t m_peer_to_be_deleted 	= PM_PEER_ID_INVALID;
static uint16_t m_conn_handle 				= BLE_CONN_HANDLE_INVALID;                  /**< Handle of the current connection. */
static int8_t	tx_power					= TX_POWER_LEVEL;							/**< Tx Power passed to the program. */
static uint8_t	lbat_alert_data[]			= "LBATALERT";
#if defined(FAST_ADV)
static uint16_t	app_adv_interval 			= APP_ADV_FAST_INTERVAL;					/**< Advertising interval passed to the program. */
#else
static uint16_t app_adv_interval			= APP_ADV_SLOW_INTERVAL;
#endif
static bool 	app_adv_param_update 		= false;
static bool		app_tx_power_param_update 	= false;
static bool		app_flash_update_state 		= false;
static bool		is_bond_deleted				= false;
static bool		is_dfu_disconnect			= false;
static bool		is_advertising				= false;
static bool		is_low_battery				= false;
static uint8_t	passkey[]					= "703819";

// Local variables in the saadc part
#define SAMPLES_IN_BUFFER 	 				1
#define SAADC_SAMPLE_INTERVAL				APP_TIMER_TICKS(1000)
#define BAT_MAX_VOLTAGE						3
#define BAT_MIN_VOLTAGE						2.143
#define SAADC_BAT_MAX_VALUE					852.5
#define SAADC_BAT_MIN_VALUE					608.969
#define SAADC_BAT_DELTA						243.531
static nrf_saadc_value_t     				m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint8_t				 				adc_value;
static uint16_t	saadc_sampling_hours 		= 3600;
static bool		saadc_sampling_first_time	= true;


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_BCS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
};

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                          /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

beacon_conf_flash_t	beacon_conf = 
{
	.uuid_major_minor = {APP_BEACON_UUID, APP_MAJOR_VALUE, APP_MINOR_VALUE},
	.rssi_ref_stored = APP_MEASURED_RSSI,
	.tx_power_stored = TX_POWER_LEVEL,
#if defined(FAST_ADV)
	.advertising_interval_stored = APP_ADV_FAST_INTERVAL
#else
	.advertising_interval_stored = APP_ADV_SLOW_INTERVAL
#endif
};

static void advertising_init(void);
static void advertising_start(bool erase_bonds);
void saadc_init(void);


/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            
            is_dfu_disconnect = true;
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            is_dfu_disconnect = false;
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
			is_dfu_disconnect = false;
            break;
    }
}

/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(uint8_t new_level)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = new_level;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void lbat_timeout_handler(void * p_context)
{
	ret_code_t err_code;
	UNUSED_PARAMETER(p_context);

	err_code = sd_ble_gap_adv_stop();
	if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
	is_low_battery = false;
	
	advertising_init();
	err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);
}

static void saadc_timeout_handler(void * p_context)
{
	ret_code_t err_code;

	UNUSED_PARAMETER(p_context);

	if(saadc_sampling_first_time == true || (--saadc_sampling_hours) == 0)
	{
		saadc_sampling_hours = 3600;
		saadc_sampling_first_time = false;
		
		saadc_init();
		nrf_drv_saadc_sample();
	}
	//bsp_board_led_invert(BSP_BOARD_LED_1);

	err_code = app_timer_start(m_saadc_timer_id, SAADC_SAMPLE_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
		float adc_temp_value;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

		//bsp_board_led_invert(BSP_BOARD_LED_1);

		adc_temp_value = p_event->data.done.p_buffer[0];
		adc_value = (uint8_t)((adc_temp_value - SAADC_BAT_MIN_VALUE)/SAADC_BAT_DELTA * 100);

		// Update battery service characteristic
		battery_level_update(adc_value);

		// Close saadc peripheral
		nrf_drv_saadc_uninit();
		NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);
		NVIC_ClearPendingIRQ(SAADC_IRQn);

		//Check whether it reaches battery low alert
		if(adc_value <= 20 && m_conn_handle == BLE_CONN_HANDLE_INVALID)
		{
			err_code = sd_ble_gap_adv_stop();
			if (err_code != NRF_ERROR_INVALID_STATE)
    		{
        		APP_ERROR_CHECK(err_code);
    		}
			is_low_battery = true;
			advertising_init();
			err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);

			err_code = app_timer_start(m_lbat_timer_id, LBATALERT_TIMEOUT_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
		}
    }
}


static void load_uicr_configuration(void)
{
#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
	// If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
	// UICR instead of using the default values. The major and minor values obtained from the UICR
	// are encoded into advertising data in big endian order (MSB First).
	// To set the UICR used by this example to a desired value, write to the address 0x10001080
	// using the nrfjprog tool. The command to be used is as follows.
	// nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
	// For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
	// the following command should be used.
	// nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
	uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
	uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

	uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

	m_beacon_info[index++] = MSB_16(major_value);
	m_beacon_info[index++] = LSB_16(major_value);

	m_beacon_info[index++] = MSB_16(minor_value);
	m_beacon_info[index++] = LSB_16(minor_value);

	memcpy(&beacon_conf.uuid_major_minor[16], &m_beacon_info[MAJ_VAL_OFFSET_IN_BEACON_INFO], 4);
#endif
}

/**@brief Function for reconfigure bsp user keys.
 *
 * @param[in] void
 */
static void leds_buttons_configure()
{
	uint32_t	err_code;

	err_code = bsp_event_to_button_action_assign(BTN_ID_LEDS_SWITCH, BTN_ACTION_LEDS_SWITCH_ON, BSP_EVENT_KEY_1);
	APP_ERROR_CHECK(err_code);
	
	err_code = bsp_event_to_button_action_assign(BTN_ID_LEDS_SWITCH, BTN_ACTION_LEDS_SWITCH_OFF, BSP_EVENT_KEY_2);
	APP_ERROR_CHECK(err_code);

	err_code = bsp_event_to_button_action_assign(BTN_ID_LBAT_TEST_SWITCH, BTN_ACTION_LBAT_TRIGGER_ON, BSP_EVENT_KEY_3);
	APP_ERROR_CHECK(err_code);
	
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
			// Start Security Request timer.
			err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            pm_conn_sec_status_t conn_sec_status;

            // Check if the link is authenticated (meaning at least MITM).
            err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
            APP_ERROR_CHECK(err_code);

            if (conn_sec_status.mitm_protected)
            {
				is_bond_deleted = false;
                NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d",
                             ble_conn_state_role(p_evt->conn_handle),
                             p_evt->conn_handle,
                             p_evt->params.conn_sec_succeeded.procedure);
            }
            else
            {
                // The peer did not use MITM, disconnect.
                NRF_LOG_INFO("Collector did not use MITM, disconnecting");
                err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            NRF_LOG_INFO("Failed to secure connection. Disconnecting.");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            //advertising_start(false);
            is_bond_deleted = true;
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for handling the Security Request timer timeout.
 *
 * @details This function will be called each time the Security Request timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    ret_code_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Initiate bonding.
        NRF_LOG_DEBUG("Start encryption");
        err_code = pm_conn_secure(m_conn_handle, false);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    // Create Security Request timer.
    err_code = app_timer_create(&m_sec_req_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sec_req_timeout_handler);
    APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_saadc_timer_id,
								APP_TIMER_MODE_SINGLE_SHOT,
								saadc_timeout_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_lbat_timer_id, 
								APP_TIMER_MODE_SINGLE_SHOT, 
								lbat_timeout_handler);
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
	ble_opt_t				ble_opt;

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */
    ble_opt.gap_opt.passkey.p_passkey = &passkey[0];

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

	err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
	APP_ERROR_CHECK(err_code);

	// Retrieved tx power value from flash/default configuration.
	tx_power = beacon_conf.tx_power_stored;
	err_code = sd_ble_gap_tx_power_set(tx_power);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Beacon Service.
 *
 * @details This function will process the data received from the Beacon BLE Service
 *
 * @param[in] p_evt    Beacon Service event structure.
 */
/**@snippet [Handling the data received over BLE] */
static void bcs_data_handler(ble_bcs_evt_t * p_evt)
{
	uint8_t   * p_data = NULL;
	uint16_t 	data_len = p_evt->params.rx_data.length;

    if (p_evt->type == BLE_BCS_EVT_ADV_RX_DATA && data_len == 21)
    {
    	p_data = (uint8_t *)p_evt->params.rx_data.p_data;
    	memcpy(&m_beacon_info[2], p_data, 21);
		memcpy(&beacon_conf.uuid_major_minor, p_data, 21);
		app_adv_param_update = true;
		app_flash_update_state = true;
    }
	else if(p_evt->type == BLE_BCS_EVT_ADVINT_RX_DATA && data_len == sizeof(uint16_t))
	{
		p_data = (uint8_t *)p_evt->params.rx_data.p_data;
		app_adv_interval = (uint16_t)(*(p_data+1) << 8) + *p_data;
		beacon_conf.advertising_interval_stored = app_adv_interval;
		app_adv_param_update = true;
		app_flash_update_state = true;
	}
	else
	{
		app_adv_param_update = false;
		// Nothing to do here.
	}
	p_data = NULL;

}

static void tps_data_handler(ble_tps_evt_t * p_evt)
{
	uint8_t * p_data = NULL;
	uint8_t   data_len = p_evt->params.rx_data.length;

	if(p_evt->evt_type == BLE_TPS_EVT_POWER_CHANGE && data_len == 1)
	{
		p_data = (uint8_t *)p_evt->params.rx_data.p_data;
		tx_power = (int8_t)*p_data;
		beacon_conf.tx_power_stored = tx_power;
		app_tx_power_param_update = true;
		app_flash_update_state = true;
	}
	else
	{
		app_tx_power_param_update = false;
		// Do nothing here.
	}
	p_data = NULL;
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	ret_code_t			err_code;
    ble_bcs_init_t		bcs_init;
	ble_bas_init_t		bas_init;
	ble_tps_init_t		tps_init;
	ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_evt_handler
    };

	// Initialize Beacon Service
	memset(&bcs_init, 0, sizeof(bcs_init));

	bcs_init.init_advint_value = app_adv_interval;
	bcs_init.data_handler = bcs_data_handler;

	err_code = ble_bcs_init(&m_bcs, &bcs_init);
	APP_ERROR_CHECK(err_code);

	// Initialize Tx Power Service
	memset(&tps_init, 0, sizeof(tps_init));

	tps_init.initial_tx_power_level = tx_power;
	tps_init.data_handler = tps_data_handler;
	
	BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&tps_init.tps_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&tps_init.tps_attr_md.write_perm);

	err_code = ble_tps_init(&m_tps, &tps_init);
	APP_ERROR_CHECK(err_code);

	// Initialize the Battery Service
	memset(&bas_init, 0, sizeof(bas_init));

	// Here the sec level for the Battery Service can be changed/increased.
	BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&bas_init.battery_level_char_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&bas_init.battery_level_char_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&bas_init.battery_level_report_read_perm);

	bas_init.evt_handler = NULL;
	bas_init.support_notification = true;
	bas_init.p_report_ref = NULL;
	bas_init.initial_batt_level = 100;

	err_code = ble_bas_init(&m_bas, &bas_init);
	APP_ERROR_CHECK(err_code);

	// Initialize the async SVCI interface to bootloader.
#ifdef DFU_SERVICE
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);


    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

	ret_code_t	err_code;

	err_code = app_timer_start(m_saadc_timer_id, SAADC_SAMPLE_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
			is_advertising = true;
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
			is_advertising = false;
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
	//uint8_t	peer_count = 0;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
			m_conn_handle = BLE_CONN_HANDLE_INVALID;
			// Turn off connection indication LED.
			err_code = bsp_indication_set(BSP_INDICATE_IDLE);
			APP_ERROR_CHECK(err_code);
            // LED indication will be changed when advertising starts.
            /*
            if((peer_count = pm_peer_count()) > 0 && is_dfu_disconnect == false)
			{
				err_code = pm_peers_delete();
				APP_ERROR_CHECK(err_code);
				err_code = pm_whitelist_set(NULL, 0);
				APP_ERROR_CHECK(err_code);
			}
			*/
			if(app_flash_update_state)
			{
				app_flash_update_state = false;
				err_code = bdef_file_update((uint8_t *)&beacon_conf, sizeof(beacon_conf));
				APP_ERROR_CHECK(err_code);
			}
			if(app_tx_power_param_update == true && m_conn_handle == BLE_CONN_HANDLE_INVALID)
			{
				app_tx_power_param_update = false;
				err_code = sd_ble_gap_tx_power_set(tx_power);
				APP_ERROR_CHECK(err_code);
				err_code = ble_tps_tx_power_level_set(&m_tps, tx_power);
				APP_ERROR_CHECK(err_code);
			}
			if(app_adv_param_update == true && m_conn_handle == BLE_CONN_HANDLE_INVALID)
			{
				app_adv_param_update = false;
				advertising_init();
			}

			err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
			
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
			m_peer_to_be_deleted = PM_PEER_ID_INVALID;
			is_advertising = false;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			// Start Security Request timer.
            err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
            break;

#ifndef S140
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
#endif

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

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

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


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

		// The LED switching events are assigned to BUTTON_2
		case BSP_EVENT_KEY_1:
			if((is_advertising == true) && (m_conn_handle == BLE_CONN_HANDLE_INVALID))
			{
				err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
				APP_ERROR_CHECK(err_code);
			}
			break;

		case BSP_EVENT_KEY_2:
			err_code = bsp_indication_set(BSP_INDICATE_IDLE);
			APP_ERROR_CHECK(err_code);
			break;

		case BSP_EVENT_KEY_3:
			if(m_conn_handle == BLE_CONN_HANDLE_INVALID)
			{
				err_code = sd_ble_gap_adv_stop();
				if (err_code != NRF_ERROR_INVALID_STATE)
    			{
        			APP_ERROR_CHECK(err_code);
    			}
				is_low_battery = true;
				advertising_init();
				err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
				APP_ERROR_CHECK(err_code);

				err_code = app_timer_start(m_lbat_timer_id, LBATALERT_TIMEOUT_INTERVAL, NULL);
				APP_ERROR_CHECK(err_code);
			}

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

#ifdef USE_IBEACON_PROTOCOL
    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

	// Set advertising property with configuration in flash.
	// If the factory setting is different with the one read from flash,
	// then use the setting from flash
	if(memcmp(&m_beacon_info[2], &beacon_conf.uuid_major_minor, 21) != 0)
	{
		memcpy(&m_beacon_info[2], &beacon_conf.uuid_major_minor, 21);
	}
	// Retrieved advertising interval from flash/default configuration
	app_adv_interval = beacon_conf.advertising_interval_stored;

	if(!is_low_battery)
	{
    	manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    	manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;
	}
	else
	{
		manuf_specific_data.data.p_data = lbat_alert_data;
		manuf_specific_data.data.size = sizeof(lbat_alert_data);
	}

    init.advdata.name_type               = BLE_ADVDATA_NO_NAME;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.p_manuf_specific_data   = &manuf_specific_data;

	init.srdata.name_type				 = BLE_ADVDATA_FULL_NAME;
	init.srdata.uuids_complete.uuid_cnt  = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	init.srdata.uuids_complete.p_uuids   = m_adv_uuids;
#else
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
#endif

    init.config.ble_adv_fast_enabled  = true;
	if(!is_low_battery)
    	init.config.ble_adv_fast_interval = app_adv_interval;
	else
		init.config.ble_adv_fast_interval = LBATALERT_INTERVAL;
    init.config.ble_adv_fast_timeout  = 0;
	init.config.ble_adv_on_disconnect_disabled = true;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    //bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

#if	LEDS_NUMBER > 0
	leds_buttons_configure();
#endif	// LEDS_NUMBER > 0

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    //APP_ERROR_CHECK(err_code);

    //*p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
        while(!is_bond_deleted);
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds = true;
	ret_code_t err_code;

    // Initialize.
    log_init();
    timers_init();
	power_management_init();
    buttons_leds_init(&erase_bonds);
	// Load UICR
	load_uicr_configuration();
	// Initialize BLE
    ble_stack_init();
	// Initialize FDS.
	err_code = bdef_file_setup();
	APP_ERROR_CHECK(err_code);
	// Load configuration from Flash BDEF File.
	err_code = bdef_file_load((uint8_t *)&beacon_conf, sizeof(beacon_conf));
	APP_ERROR_CHECK(err_code);
	// Initialize gap, gatt, services, advertising and pm
	peer_manager_init();
    gap_params_init();
    gatt_init();
	services_init();
    advertising_init();
    conn_params_init();
    //peer_manager_init();

    // Start execution.
    NRF_LOG_INFO("Application started.");
    application_timers_start();

    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
		/*
        if (NRF_LOG_PROCESS() == false)
        {
            //power_manage();
			nrf_pwr_mgmt_run();
        }
        */
        nrf_pwr_mgmt_run();
    }
}


/**
 * @}
 */
