/*!*****************************************************************
 * \file    cli.h
 * \brief   Command line interface.
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

#ifndef __CLI_H__
#define __CLI_H__

#include "stdint.h"

/*** CLI structures ***/

/*!******************************************************************
 * \enum CLI_status_t
 * \brief CLI driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    CLI_SUCCESS = 0,
    CLI_ERROR_NULL_PARAMETER,
    // Low level drivers errors.
    CLI_ERROR_DRIVER_AT,
    CLI_ERROR_DRIVER_SIGFOX_EP_LIB,
    CLI_ERROR_DRIVER_SIGFOX_EP_ADDON_RFP,
    CLI_ERROR_DRIVER_SIGFOX_EP_ADDON_TA,
    CLI_ERROR_DRIVER_MCAL,
    CLI_ERROR_RC_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_UL_BR_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_N_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_T_IFU_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_KEY_TYPE_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_T_CONF_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_ERROR_STACK_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_APP_MESSAGE_BYTES_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_BIDIR_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_UL_PAYLOAD_BYTES_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_UL_PAYLOAD_BIT_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_UL_PAYLOAD_EMPTY_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    CLI_ERROR_RFP_TEST_MODE_NOT_SUPPORTED_IN_THIS_CONFIGURATION,
    // Last index.
    CLI_ERROR_LAST
} CLI_status_t;

/*** CLI functions ***/

/*!******************************************************************
 * \fn CLI_status_t CLI_init(void)
 * \brief Initialize command line interface.
 * \param[in]   process_callback: Function to be called when the CLI driver has to be processed.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_init(void);

/*!******************************************************************
 * \fn CLI_status_t CLI_de_init(void)
 * \brief Release command line interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_de_init(void);

/*!******************************************************************
 * \fn CLI_status_t CLI_process(void)
 * \brief Process command line interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_process(void);

#endif /* __CLI_H__ */