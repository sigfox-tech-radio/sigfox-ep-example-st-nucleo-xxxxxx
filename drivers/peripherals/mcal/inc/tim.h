/*!*****************************************************************
 * \file    tim.h
 * \brief   Common TIM driver based on LL driver.
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

#ifndef __TIM_H__
#define __TIM_H__

#include "mcal.h"
#include "stdint.h"

/*** TIM structures ***/

/*!******************************************************************
 * \enum TIM_channel_t
 * \brief Timer channels list.
 *******************************************************************/
typedef enum {
	TIM_CHANNEL_CH1 = 0,
	TIM_CHANNEL_CH2,
	TIM_CHANNEL_CH3,
	TIM_CHANNEL_CH4,
	TIM_CHANNEL_LAST
} TIM_channel_t;

/*!******************************************************************
 * \enum TIM_waiting_mode_t
 * \brief Timer completion waiting modes.
 *******************************************************************/
typedef enum {
	TIM_WAITING_MODE_ACTIVE = 0,
	TIM_WAITING_MODE_SLEEP,
	TIM_WAITING_MODE_LOW_POWER_SLEEP,
	TIM_WAITING_MODE_LAST
} TIM_waiting_mode_t;

/*!******************************************************************
 * \fn TIM_completion_irq_cb_t
 * \brief TIM completion callback.
 *******************************************************************/
typedef void (*TIM_completion_irq_cb_t)(void);

/*** TIM functions ***/

/*!******************************************************************
 * \fn void TIM_init(void)
 * \brief Init timer driver.
 * \param[in]  	irq_priority; Interrupt priority.
 * \param[out] 	none
 * \retval		Function execution status..
 *******************************************************************/
MCAL_status_t TIM_init(uint8_t irq_priority);

/*!******************************************************************
 * \fn void TIM_de_init(void)
 * \brief Release timer driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM_de_init(void);

/*!******************************************************************
 * \fn MCAL_status_t TIM_start(TIM_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode)
 * \brief Start a timer channel.
 * \param[in]  	channel: Channel to start.
 * \param[in]	duration_ms: Timer duration in ms.
 * \param[in]	waiting_mode: Completion waiting mode.
 * \param[in]	irq_callback: Function to call on timer completion.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t TIM_start(TIM_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode, TIM_completion_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn MCAL_status_t TIM_stop(TIM_channel_t channel)
 * \brief Stop a timer channel.
 * \param[in]  	channel: Channel to stop.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t TIM_stop(TIM_channel_t channel);

/*!******************************************************************
 * \fn MCAL_status_t TIM_get_status(TIM_channel_t channel, uint8_t* timer_has_elapsed)
 * \brief Get the status of a timer channel.
 * \param[in]  	channel: Channel to read.
 * \param[out]	timer_has_elapsed: Pointer to bit that will contain the timer status (0 for running, 1 for complete).
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t TIM_get_status(TIM_channel_t channel, uint8_t* timer_has_elapsed);

/*!******************************************************************
 * \fn MCAL_status_t TIM_wait_completion(TIM_channel_t channel)
 * \brief Blocking function waiting for a timer channel completion.
 * \param[in]  	channel: Channel to wait for.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t TIM_wait_completion(TIM_channel_t channel);

/*!******************************************************************
 * \fn MCAL_status_t TIM_clear_irq_flag(TIM_channel_t channel)
 * \brief Clear timer channel interrupt flag.
 * \param[in]  	channel: Channel to clear.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t TIM_clear_irq_flag(TIM_channel_t channel);

#endif /* __TIM_H__ */
