/*!*****************************************************************
 * \file    lptim.h
 * \brief   Common LPTIM driver based on LL driver.
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

#ifndef __LPTIM_H__
#define __LPTIM_H__

#include "mcal.h"
#include "stdint.h"

/*** LPTIM structures ***/

/*!******************************************************************
 * \enum LPTIM_delay_mode_t
 * \brief LPTIM delay waiting modes.
 *******************************************************************/
typedef enum {
	LPTIM_DELAY_MODE_ACTIVE = 0,
	LPTIM_DELAY_MODE_SLEEP,
	LPTIM_DELAY_MODE_STOP,
	LPTIM_DELAY_MODE_LAST
} LPTIM_delay_mode_t;

/*** LPTIM functions ***/

/*!******************************************************************
 * \fn MCAL_status_t LPTIM_init(void)
 * \brief Init LPTIM1 peripheral for delay operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t LPTIM_init(void);

/*!******************************************************************
 * \fn MCAL_status_t LPTIM_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode)
 * \brief Delay function.
 * \param[in]  	delay_ms: Delay to wait in ms.
 * \param[in]	delay_mode: Delay waiting mode.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t LPTIM_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode);

#endif /* __LPTIM_H__ */
