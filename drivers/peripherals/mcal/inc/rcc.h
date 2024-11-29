/*!*****************************************************************
 * \file    rcc.h
 * \brief   Common RCC driver based on LL driver.
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

#ifndef __RCC_H__
#define __RCC_H__

#include "mcal.h"
#include "stdint.h"

/*** RCC structures ***/

/*!******************************************************************
 * \enum RCC_msi_range_t
 * \brief RCC MSI oscillator frequency ranges.
 *******************************************************************/
typedef enum {
    RCC_MSI_RANGE_0_65KHZ = 0,
    RCC_MSI_RANGE_1_131KHZ,
    RCC_MSI_RANGE_2_262KHZ,
    RCC_MSI_RANGE_3_524KKZ,
    RCC_MSI_RANGE_4_1MHZ,
    RCC_MSI_RANGE_5_2MHZ,
    RCC_MSI_RANGE_6_4MHZ,
    RCC_MSI_RANGE_LAST
} RCC_msi_range_t;

/*** RCC functions ***/

/*!******************************************************************
 * \fn void RCC_init(void)
 * \brief Init MCU clock tree.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status..
 *******************************************************************/
MCAL_status_t RCC_init(void);

/*!******************************************************************
 * \fn RCC_status_t RCC_switch_to_hsi(void)
 * \brief Switch system clock to 16MHz HSI.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t RCC_switch_to_hsi(void);

/*!******************************************************************
 * \fn RCC_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range)
 * \brief Switch system clock to MSI.
 * \param[in]   msi_range: MSI frequency to set.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range);

#endif /* __RCC_H__ */
