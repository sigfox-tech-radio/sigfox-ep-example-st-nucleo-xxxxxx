/*!*****************************************************************
 * \file    shield_sx1262mb2cas.h
 * \brief   SX1262MB2CAS shield configuration.
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

#ifndef __SX126X_SHIELD_SX1262MB2CAS_H__
#define __SX126X_SHIELD_SX1262MB2CAS_H__

#include "sx126x_mapping.h"
#include "board/sx126x_hw_api.h"

/*** SHIELD SX1262MB2CAS functions ***/

/*!******************************************************************
 * \fn const SX126X_HW_API_pa_pwr_cfg_t *SHIELD_SX1262MB2CAS_get_pa_pwr_cfg(const sfx_u32 rf_freq_in_hz, sfx_s8 expected_output_pwr_in_dbm);
 * \brief Get the PA power configuration.
 * \param[in]   rf_freq_in_hz: RF frequency in Hz.
 * \param[in]   expected_output_pwr_in_dbm: Expected output power in dBm.
 * \param[out]  none
 * \retval      Pointer to PA power configuration structure.
 *******************************************************************/
const SX126X_HW_API_pa_pwr_cfg_t SHIELD_SX1262MB2CAS_get_pa_pwr_cfg(const sfx_u32 rf_freq_in_hz, sfx_s8 expected_output_pwr_in_dbm);

/*!******************************************************************
 * \fn sfx_bool SHIELD_SX1262MB2CAS_is_dio2_set_as_rf_switch(void);
 * \brief Check if DIO2 is set as RF switch.
 * \param[in]   none
 * \param[out]  none
 * \retval      TRUE if DIO2 is set as RF switch, FALSE otherwise.
 *******************************************************************/
const SX126X_HW_API_pa_pwr_cfg_t SHIELD_SX1262MB2CAS_is_dio2_set_as_rf_switch(void);

/*!******************************************************************
 * \fn const SX126X_HW_API_xosc_cfg_t *SHIELD_SX1262MB2CAS_get_xosc_cfg(void);
 * \brief Get the XOSC configuration.
 * \param[in]   none
 * \param[out]  none
 * \retval      Pointer to oscillator structure.
 *******************************************************************/
SX126X_HW_API_reg_mod_t SHIELD_SX1262MB2CAS_get_reg_mode(void);

/*!******************************************************************
 * \fn const SX126X_HW_API_xosc_cfg_t *SHIELD_SX1262MB2CAS_get_xosc_cfg(void);
 * \brief Get the XOSC configuration.
 * \param[in]   none
 * \param[out]  none
 * \retval      Pointer to oscillator structure.
 *******************************************************************/
const SX126X_HW_API_xosc_cfg_t *SHIELD_SX1262MB2CAS_get_xosc_cfg(void);

/*!******************************************************************
 * \fn const SX126X_MAPPING_gpios_t *SHIELD_SX1262MB2CAS_get_pinout(void);
 * \brief Get the SX1262MB2CAS shield pinout.
 * \param[in]   none
 * \param[out]  none
 * \retval      Pointer to SX1262MB2CAS shield pinout structure.
 *******************************************************************/
const SX126X_MAPPING_gpios_t *SHIELD_SX1262MB2CAS_get_pinout(void);

#endif // __SX126X_SHIELD_SX1262MB2CAS_H__