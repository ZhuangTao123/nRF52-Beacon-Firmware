#ifndef BDEF_FILE_M_H__
#define BDEF_FILE_M_H__

#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"
#include "sdk_errors.h"
#include "nrf.h"

// Define beacon configuration structure stored in flash
typedef struct
{
	uint8_t		uuid_major_minor[20];
	int8_t		rssi_ref_stored;
	int8_t		tx_power_stored;
	uint16_t	advertising_interval_stored;
}__attribute__((packed)) beacon_conf_flash_t;

/**
 * @brief   Function for initializing the FDS module.
 *
 * @return  NRF_SUCCESS when module has been set up properly,
 *          error code otherwise.
 */
ret_code_t bdef_file_setup(void);

/**
 * @brief   Function for updating BDEF message in the flash file.
 *
 * @details FDS update operation is performed asynchronously,
 *          operation status is reported through events.
 *
 * @param[in] p_buff Pointer to the BDEF message to be stored in flash.
 * @param[in] size   Size of NDEF message.
 *
 * @return  NRF_SUCCESS when update request has been added to the queue.
 *          Otherwise, FDS error code.
 */
ret_code_t bdef_file_update(uint8_t const * p_buff, uint32_t size);

/**
 * @breif   Function for loading BDEF message from the flash file.
 *
 * @details If the flash file does not exist, the default BDEF message
 *          is created and stored in flash.
 *
 * @param[out] p_buff Pointer to the buffer for BDEF message.
 * @param[in]  size   Size of the buffer.
 *
 * @return  NRF_SUCCESS when BDEF message has been retrieved properly.
 *          Otherwise, FDS error code.
 */
ret_code_t bdef_file_load(uint8_t * p_buff, uint32_t size);


/** @} */


#endif // BDEF_FILE_M_H__

