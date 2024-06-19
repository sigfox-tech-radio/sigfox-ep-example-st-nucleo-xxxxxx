/*!*****************************************************************
 * \file    mcu_api.c
 * \brief   MCU drivers.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "manuf/mcu_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#include "adc.h"
#include "mcal.h"
#include "nvic.h"
#include "nvm.h"
#include "stddef.h"
#include "stdint.h"
#include "TI_aes_128_encr_only.h"
#include "tim.h"

/*** MCU API local global variables ***/

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION) && (defined BIDIRECTIONAL)
static sfx_u32 MCU_API_LATENCY_MS[MCU_API_LATENCY_LAST] = {
	0 // Get voltage and temperature function (65us).
};
#endif
#ifdef VERBOSE
static const sfx_u8 MCU_API_VERSION[] = "v1.0";
#endif

const uint8_t sigfoxID[4] __attribute__ ((section(".sigfoxID"))) = {0xFE, 0xDC, 0xBA,  0x98};
const uint8_t sigfoxKEY[16] __attribute__ ((section(".sigfoxKEY"))) =
                        {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

/*** MCU API local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) EXIT_ERROR(MCU_API_ERROR); }

/*** MCU API functions ***/

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
MCU_API_status_t MCU_API_open(MCU_API_config_t *mcu_api_config) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Ignore unused parameters.
	MCAL_UNUSED(mcu_api_config);
	// Init timer.
	mcal_status = TIM_init(NVIC_IRQ_PRIORITY_TIM);
	_check_mcal_status();
#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
	// Init ADC.
	mcal_status = ADC_init();
	_check_mcal_status();
#endif
errors:
	RETURN();
}
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
MCU_API_status_t MCU_API_close(void) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
	MCAL_status_t mcal_status = MCAL_SUCCESS;
#endif
	// Release timer.
	TIM_de_init();
#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
	// Release ADC.
	mcal_status = ADC_de_init();
	_check_mcal_status();
errors:
#endif
	RETURN();
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
MCU_API_status_t MCU_API_process(void) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef TIMER_REQUIRED
/*******************************************************************/
MCU_API_status_t MCU_API_timer_start(MCU_API_timer_t *timer) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	TIM_waiting_mode_t tim_waiting_mode = TIM_WAITING_MODE_LOW_POWER_SLEEP;
	TIM_completion_irq_cb_t irq_callback = NULL;
	// Check parameter.
	if (timer == SFX_NULL) {
		EXIT_ERROR(MCU_API_ERROR);
	}
#ifdef BIDIRECTIONAL
	// Update waiting mode according to timer reason.
	if ((timer -> reason) == MCU_API_TIMER_REASON_T_RX) {
		// T_RX completion is directly checked with the raw timer status within the RF_API_receive() function.
		// All other timers completion are checked with the MCU_API_timer_wait_cplt() function, using low power sleep waiting mode.
		tim_waiting_mode = TIM_WAITING_MODE_ACTIVE;
	}
#endif
#ifdef ASYNCHRONOUS
	irq_callback = (timer -> cplt_cb);
#endif
	// Start timer.
	mcal_status = TIM_start(((TIM_channel_t) (timer -> instance)), (timer -> duration_ms), tim_waiting_mode, irq_callback);
	_check_mcal_status();
errors:
	RETURN();
}
#endif

#ifdef TIMER_REQUIRED
/*******************************************************************/
MCU_API_status_t MCU_API_timer_stop(MCU_API_timer_instance_t timer_instance) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Stop timer.
	mcal_status = TIM_stop((TIM_channel_t) timer_instance);
	_check_mcal_status();
errors:
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
/*******************************************************************/
MCU_API_status_t MCU_API_timer_status(MCU_API_timer_instance_t timer_instance, sfx_bool *timer_has_elapsed) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Read status.
	mcal_status = TIM_get_status(((TIM_channel_t) timer_instance), timer_has_elapsed);
	_check_mcal_status();
errors:
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
/*******************************************************************/
MCU_API_status_t MCU_API_timer_wait_cplt(MCU_API_timer_instance_t timer_instance) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Read status.
	mcal_status = TIM_wait_completion((TIM_channel_t) timer_instance);
	_check_mcal_status();
errors:
	RETURN();
}
#endif

/*******************************************************************/
MCU_API_status_t MCU_API_aes_128_cbc_encrypt(MCU_API_encryption_data_t *aes_data) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	uint8_t local_key[SIGFOX_EP_KEY_SIZE_BYTES];
	uint8_t idx = 0;
	// Get right key.
#ifdef PUBLIC_KEY_CAPABLE
	switch (aes_data -> key) {
	case SIGFOX_EP_KEY_PRIVATE:
        for (idx = 0 ; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
            local_key[idx] = sigfoxKEY[idx];
        }
		break;
	case SIGFOX_EP_KEY_PUBLIC:
		// Use public key.
		for (idx=0 ; idx<SIGFOX_EP_KEY_SIZE_BYTES ; idx++) {
			local_key[idx] = SIGFOX_EP_PUBLIC_KEY[idx];
		}
		break;
	default:
		EXIT_ERROR(MCU_API_ERROR);
		break;
	}
#else
	// Retrieve private key from NVM.
    for (idx = 0 ; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
        local_key[idx] = sigfoxKEY[idx];
    }
#endif
	// Init peripheral.
	aes_encrypt((aes_data -> data), (uint8_t*) local_key);
#ifdef PUBLIC_KEY_CAPABLE
errors:
#endif
	RETURN();
}

#ifdef CRC_HW
/*******************************************************************/
MCU_API_status_t MCU_API_compute_crc16(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined CRC_HW) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_compute_crc8(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u8 *crc) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

/*******************************************************************/
MCU_API_status_t MCU_API_get_ep_id(sfx_u8 *ep_id, sfx_u8 ep_id_size_bytes) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	uint8_t idx = 0;
	// Get device ID.
	for (idx=0 ; idx<ep_id_size_bytes ; idx++) {
	    ep_id[idx] = sigfoxID[idx];
	}
	RETURN();
}

/*******************************************************************/
MCU_API_status_t MCU_API_get_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	uint8_t idx = 0;
	// Read data.
	for (idx=0 ; idx<nvm_data_size_bytes ; idx++) {
		mcal_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_EP_LIB_DATA + idx), &(nvm_data[idx]));
		_check_mcal_status();
	}
errors:
	RETURN();
}

/*******************************************************************/
MCU_API_status_t MCU_API_set_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	uint8_t idx = 0;
	// Write data.
	for (idx=0 ; idx<nvm_data_size_bytes ; idx++) {
		mcal_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_EP_LIB_DATA + idx), nvm_data[idx]);
		_check_mcal_status();
	}
errors:
	RETURN();
}

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle_mv, sfx_u16 *voltage_tx_mv, sfx_s16 *temperature_tenth_degrees) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	// Local variables.
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	uint16_t mcu_supply_voltage_mv = 0;
	int16_t mcu_temperature_degrees = 0;
	// Perform analog measurements.
	mcal_status = ADC_get_mcu_voltage(&mcu_supply_voltage_mv);
	_check_mcal_status();
	// Get MCU supply voltage.
	(*voltage_idle_mv) = (sfx_u16) mcu_supply_voltage_mv;
	(*voltage_tx_mv) = (sfx_u16) mcu_supply_voltage_mv;
	// Get MCU internal temperature.
	mcal_status = ADC_get_mcu_temperature(&mcu_temperature_degrees);
	_check_mcal_status();
	(*temperature_tenth_degrees) = ((sfx_s16) mcu_temperature_degrees) * 10;
errors:
	RETURN();
}
#endif

#if (defined CERTIFICATION) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	SFX_UNUSED(dl_payload);
	SFX_UNUSED(dl_payload_size);
	SFX_UNUSED(rssi_dbm);
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
MCU_API_status_t MCU_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	MCAL_UNUSED(initial_pac);
	MCAL_UNUSED(initial_pac_size_bytes);
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_get_latency(MCU_API_latency_t latency_type, sfx_u32 *latency_ms) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	// Check parameters.
	if ((latency_type >= MCU_API_LATENCY_LAST) || (latency_ms == NULL)) {
		EXIT_ERROR(MCU_API_ERROR);
	}
	// Set latency.
	(*latency_ms) = MCU_API_LATENCY_MS[latency_type];
errors:
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
MCU_API_status_t MCU_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
	// Local variables.
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	// Check parameters.
	if ((version == NULL) || (version_size_char == NULL)) {
		EXIT_ERROR(MCU_API_ERROR);
	}
	(*version) = (sfx_u8*) MCU_API_VERSION;
	(*version_size_char) = (sfx_u8) sizeof(MCU_API_VERSION);
errors:
	RETURN();
}
#endif

#ifdef ERROR_CODES
/*******************************************************************/
void MCU_API_error(void) {
	TIM_de_init();
	ADC_de_init();
}
#endif
