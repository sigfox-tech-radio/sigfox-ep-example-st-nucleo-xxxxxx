/*!*****************************************************************
 * \file    cli.c
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

#include "cli.h"

#include "at.h"
#include "mcal.h"
#include "mcu.h"
#include "nvm.h"
#include "sigfox_ep_api.h"
#include "sigfox_ep_addon_rfp_api.h"
#include "sigfox_ep_addon_ta_api.h"
#include "sigfox_ep_flags.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"
#include "stddef.h"
#include "stdio.h"
#include "string.h"
#include "manuf/rf_api.h"

/*** CLI local macros ***/

#define CLI_FIRMWARE_VERSION        "v2.1"

#define CLI_REPLY_BUFFER_SIZE       256

#define DEFAULT_ANTENNA_GAIN_DBI   2

/*** CLI local structures ***/

/*******************************************************************/
typedef enum {
    CLI_INFORMATION_INDEX_CHIP_ID = 0,
    CLI_INFORMATION_INDEX_SIGFOX_EP_ID,
    CLI_INFORMATION_INDEX_FIRMWARE_VERSION,
    CLI_INFORMATION_INDEX_LAST
} CLI_information_index_t;

/*******************************************************************/
typedef struct {
    const char *name;
    const SIGFOX_rc_t *ptr;
    const SIGFOX_ul_bit_rate_t default_sigfox_ul_bit_rate;
} CLI_sigfox_rc_t;

/*******************************************************************/
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
typedef struct {
    const char *name;
    SIGFOX_ep_key_t ep_key_type;
} CLI_ep_key_type_t;
#endif /* SIGFOX_EP_PRIVATE_KEY */

/*******************************************************************/
typedef struct {
    const char *letter;
    SIGFOX_EP_ADDON_RFP_API_test_mode_reference_t test_mode_reference;
} CLI_sigfox_ep_addon_rfp_test_mode_t;

/*******************************************************************/
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_FH
typedef struct {
    const char *name;
    SIGFOX_EP_ADDON_TA_API_fh_mode_t fh_mode;
} CLI_fh_mode_t;
#endif /* SIGFOX_EP_SPECTRUM_ACCESS_FH */

/*******************************************************************/
typedef union {
    struct {
        uint8_t at_process :1;
    } field;
    uint8_t all;
} CLI_flags_t;

/*******************************************************************/
typedef struct {
    volatile CLI_flags_t flags;
    SIGFOX_EP_API_config_t lib_config;
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    SIGFOX_EP_API_application_message_t application_message;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    SIGFOX_EP_API_control_message_t control_message;
#endif /* SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE */
} CLI_context_t;

/*** CLI local functions declaration ***/

static const char *_cli_error_enum_to_str_callback(unsigned int error_code);

static AT_status_t _info_execution_callback(int32_t *error_code);
static AT_status_t _info_write_callback(uint32_t argc, char *argv[], int32_t *error_code);

static AT_status_t _rc_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code);
static AT_status_t _rc_read_callback(CLI_status_t *error_code);

static AT_status_t _ul_br_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code);
static AT_status_t _ul_br_read_callback(CLI_status_t *error_code);

static AT_status_t _n_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code);
static AT_status_t _n_read_callback(CLI_status_t *error_code);

static AT_status_t _t_ifu_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code);
static AT_status_t _t_ifu_read_callback(CLI_status_t *error_code);

static AT_status_t _key_type_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code);
static AT_status_t _key_type_read_callback(CLI_status_t *error_code);

static AT_status_t _t_conf_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code);
static AT_status_t _t_conf_read_callback(CLI_status_t *error_code);

static AT_status_t _errors_read_callback(CLI_status_t *error_code);

static AT_status_t _msg_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code);

static AT_status_t _rfp_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code);

/*** CLI local global variables ***/

static const char *CLI_STATUS_STR[] = {
    // Driver errors.
    "CLI_SUCCESS",
    "NULL_PARAMETER",
    // Low level drivers errors.
    "DRIVER_AT",
    "DRIVER_SIGFOX_EP_LIB",
    "DRIVER_SIGFOX_EP_ADDON_RFP",
    "DRIVER_SIGFOX_EP_ADDON_TA",
    "DRIVER_MCAL",
    "RC_NOT_SUPPORTED",
    "UL_BR_NOT_SUPPORTED",
    "N_NOT_SUPPORTED"
    "T_IFU_NOT_SUPPORTED",
    "KEY_TYPE_NOT_SUPPORTED",
    "T_CONF_NOT_SUPPORTED",
    "STACK_NOT_SUPPORTED",
    "APP_MESSAGE_BYTES_NOT_SUPPORTED",
    "BIDIR_NOT_SUPPORTED",
    "UL_PAYLOAD_BYTES_NOT_SUPPORTED",
    "UL_PAYLOAD_BIT_NOT_SUPPORTED",
    "UL_PAYLOAD_EMPTY_NOT_SUPPORTED",
    "RFP_TEST_MODE_NOT_SUPPORTED",
    // Last index.
    "ERROR_LAST",
};

static CLI_sigfox_rc_t CLI_SIGFOX_RC[] = {
#ifdef SIGFOX_EP_RC1_ZONE
    {"RC1", &SIGFOX_RC1, SIGFOX_UL_BIT_RATE_100BPS},
#else
    {"RC1", NULL, 0},
#endif /* SIGFOX_EP_RC1_ZONE */
#ifdef SIGFOX_EP_RC2_ZONE
    {"RC2", &SIGFOX_RC2, SIGFOX_UL_BIT_RATE_600BPS},
#else
    {"RC2", NULL, 0},
#endif /* SIGFOX_EP_RC2_ZONE */
#ifdef SIGFOX_EP_RC3_LBT_ZONE
    {"RC3_LBT", &SIGFOX_RC3_LBT, SIGFOX_UL_BIT_RATE_100BPS},
#else
    {"RC3_LBT", NULL, 0},
#endif /* SIGFOX_EP_RC3_LBT_ZONE */
#ifdef SIGFOX_EP_RC3_LDC_ZONE
    {"RC3_LDC", &SIGFOX_RC3_LDC, SIGFOX_UL_BIT_RATE_100BPS},
#else
    {"RC3_LDC", NULL, 0},
#endif /* SIGFOX_EP_RC3_LDC_ZONE */
#ifdef SIGFOX_EP_RC4_ZONE
    {"RC4", &SIGFOX_RC4, SIGFOX_UL_BIT_RATE_600BPS},
#else
    {"RC4", NULL, 0},
#endif /* SIGFOX_EP_RC4_ZONE */
#ifdef SIGFOX_EP_RC5_ZONE
    {"RC5", &SIGFOX_RC5, SIGFOX_UL_BIT_RATE_100BPS},
#else
    {"RC5", NULL, 0},
#endif /* SIGFOX_EP_RC5_ZONE */
#ifdef SIGFOX_EP_RC6_ZONE
    {"RC6", &SIGFOX_RC6, SIGFOX_UL_BIT_RATE_100BPS},
#else
    {"RC6", NULL, 0},
#endif /* SIGFOX_EP_RC6_ZONE */
#ifdef SIGFOX_EP_RC7_ZONE
    {"RC7", &SIGFOX_RC7, SIGFOX_UL_BIT_RATE_100BPS},
#else
    {"RC7", NULL, 0},
#endif /* SIGFOX_EP_RC7_ZONE */
};

#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
static CLI_ep_key_type_t CLI_SIGFOX_EP_KEY_TYPE[SIGFOX_EP_KEY_LAST] = {
    {"PRIVATE", SIGFOX_EP_KEY_PRIVATE},
    {"PUBLIC", SIGFOX_EP_KEY_PUBLIC},
};
#endif /* SIGFOX_EP_PRIVATE_KEY */

static const CLI_sigfox_ep_addon_rfp_test_mode_t CLI_SIGFOX_EP_ADDON_RFP_TEST_MODE[] = {
    {"A", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_A},
    {"B", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_B},
    {"C", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_C},
#ifdef SIGFOX_EP_BIDIRECTIONAL
    {"D", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_D},
    {"E", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_E},
    {"F", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_F},
#else
    {"D", 14},
    {"E", 14},
    {"F", 14},
#endif
#if ((defined SIGFOX_EP_RC3_LBT_ZONE) || (defined SIGFOX_EP_RC5_ZONE))
    {"G", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_G},
#else
    {"G", 14},
#endif
    {"J", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_J},
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    {"K", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_K},
#else
    {"K", 14},
#endif
    {"L", SIGFOX_EP_ADDON_RFP_API_TEST_MODE_L},
};

static const AT_command_t CLI_COMMAND_INFO = {
    .syntax = "I",
    .type = AT_COMMAND_TYPE_BASIC,
    .help = "Modem information",
    .execution_callback = &_info_execution_callback,
    .execution_help = "Read all informations",
    .read_callback = NULL,
    .read_help = NULL,
    .write_callback = &_info_write_callback,
    .write_arguments = "<index>",
    .write_help = "Read information <index>",
    .enum_to_str_callback = NULL,
};

static const AT_command_t CLI_COMMAND_RC = {
    .syntax = "RC",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox Radio Configuration",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = (AT_command_read_cb_t) &_rc_read_callback,
    .read_help = "Read the Sigfox Radio Configuration",
    .write_callback = (AT_command_write_cb_t) &_rc_write_callback,
    .write_arguments = "<rc>:RC1",
    .write_help = "Change the Sigfox Radio Configuration (<rc>={RC1 | RC2 | RC3_LBT | RC3_LDC | RC4 | RC5 | RC6 | RC7})",
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static const AT_command_t CLI_COMMAND_UL_BR = {
    .syntax = "UL_BR",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox U-Procedure bit rate",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = (AT_command_read_cb_t) &_ul_br_read_callback,
    .read_help = "Read the Sigfox uplink bitrate",
    .write_callback = (AT_command_write_cb_t) &_ul_br_write_callback,
    .write_arguments = "<ul_br>:100",
    .write_help = "Change the Sigfox uplink bitrate (<ul_br>={100 | 600})",
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static const AT_command_t CLI_COMMAND_N = {
    .syntax = "N",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox U-Procedure Number of frames",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = (AT_command_read_cb_t) &_n_read_callback,
    .read_help ="Read the sigfox number of frame(s)",
    .write_callback = (AT_command_write_cb_t) &_n_write_callback,
    .write_arguments = "<n>:3",
    .write_help = "Change the Sigfox number of frame(s) (<n={1 to 3})",
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static const AT_command_t CLI_COMMAND_T_IFU = {
    .syntax = "T_IFU",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox U-Procedure Inter-Frame Time in ms",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = (AT_command_read_cb_t) &_t_ifu_read_callback,
    .read_help = "Read the Sigfox uplink inter-frame time",
    .write_callback = (AT_command_write_cb_t) &_t_ifu_write_callback,
    .write_arguments = "<t_ifu>:500",
    .write_help = "Change the Sigfox uplink inter-frame time (<t_ifu>={0 to 2000})",
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static const AT_command_t CLI_COMMAND_KEY_TYPE = {
    .syntax = "KEY_TYPE",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox Switch to public or private key",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = (AT_command_read_cb_t) &_key_type_read_callback,
    .read_help = "Read the sigfox key type",
    .write_callback = (AT_command_write_cb_t) &_key_type_write_callback,
    .write_arguments = "<key_type>:PRIVATE",
    .write_help = "Change the sigfox key type (<key_type>={PRIVATE | PUBLIC})",
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static const AT_command_t CLI_COMMAND_T_CONF = {
    .syntax = "T_CONF",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox D-Procedure Confirmation message Time in ms",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = (AT_command_read_cb_t) &_t_conf_read_callback,
    .read_help = "Read the confirmation message time",
    .write_callback = (AT_command_write_cb_t) &_t_conf_write_callback,
    .write_arguments = "<t_conf>:1500",
    .write_help = "Change the sigfox key type (<t_conf>={1400 to 4000})",
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static const AT_command_t CLI_COMMAND_ERRORS = {
    .syntax = "ERRORS",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox Error Stack",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = (AT_command_read_cb_t) &_errors_read_callback,
    .read_help = "Read the Sigfox error stack",
    .write_callback = NULL,
    .write_arguments = NULL,
    .write_help = NULL,
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static const AT_command_t CLI_COMMAND_MSG = {
    .syntax = "MSG",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox U-Procedure",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = NULL,
    .read_help = NULL,
    .write_callback = (AT_command_write_cb_t) &_msg_write_callback,
    .write_arguments = "<data>,<bidir_flag>:0",
    .write_help = "Send Sigfox message (<data>={HEX | 0 | 1 | NULL |}, <bidir_flag>={'0'|'1'})",
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static const AT_command_t CLI_COMMAND_RFP = {
    .syntax = "RFP",
    .type = AT_COMMAND_TYPE_EXTENDED,
    .help = "Sigfox RF and Protocol Add-on",
    .execution_callback = NULL,
    .execution_help = NULL,
    .read_callback = NULL,
    .read_help = NULL,
    .write_callback = (AT_command_write_cb_t) &_rfp_write_callback,
    .write_arguments = "<mode>",
    .write_help = "Execute Sigfox RF and Protocol test mode  (<mode>={A | B | C | D | F | G | J | K | L})",
    .enum_to_str_callback = &_cli_error_enum_to_str_callback,
};

static CLI_context_t cli_ctx = {
    .flags.all = 0,
// Init the library configuration
#if defined(SIGFOX_EP_RC1_ZONE)
    .lib_config.rc = &SIGFOX_RC1,
#elif defined(SIGFOX_EP_RC2_ZONE)
    .lib_config.rc = &SIGFOX_RC2,
#elif defined(SIGFOX_EP_RC3_LBT_ZONE)
    .lib_config.rc = &SIGFOX_RC3_LBT,
#elif defined(SIGFOX_EP_RC3_LDC_ZONE)
    .lib_config.rc = &SIGFOX_RC3_LDC,
#elif defined(SIGFOX_EP_RC4_ZONE)
    .lib_config.rc = &SIGFOX_RC4,
#elif defined(SIGFOX_EP_RC5_ZONE)
    .lib_config.rc = &SIGFOX_RC5,
#elif defined(SIGFOX_EP_RC6_ZONE)
    .lib_config.rc = &SIGFOX_RC6,
#elif defined(SIGFOX_EP_RC7_ZONE)
    .lib_config.rc = &SIGFOX_RC7,
#endif /* SIGFOX_EP_RCx */
#ifndef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
    .lib_config.message_counter_rollover = SIGFOX_MESSAGE_COUNTER_ROLLOVER_4096,
#endif /* SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    .lib_config.process_cb = NULL,
#endif /* SIGFOX_EP_ASYNCHRONOUS */
// Init the application message configuration
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    .application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS,
#endif /* SIGFOX_EP_UL_BIT_RATE_BPS */
#ifndef SIGFOX_EP_TX_POWER_DBM_EIRP
    .application_message.common_parameters.tx_power_dbm_eirp = 14,
#endif /* SIGFOX_EP_TX_POWER_DBM_EIRP */
#ifndef SIGFOX_EP_SINGLE_FRAME
    .application_message.common_parameters.number_of_frames = 3,
#ifndef SIGFOX_EP_T_IFU_MS
    .application_message.common_parameters.t_ifu_ms = 500,
#endif /* SIGFOX_EP_T_IFU_MS */
#endif /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    .application_message.common_parameters.ep_key_type = SIGFOX_EP_KEY_PRIVATE,
#endif /* SIGFOX_EP_PUBLIC_KEY_CAPABLE */
#endif /* !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE) */
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 0)
    .application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY,
#else
    .application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY,
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#else
    .application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY,
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    .application_message.uplink_cplt_cb = NULL,
#ifdef SIGFOX_EP_BIDIRECTIONAL
    .application_message.downlink_cplt_cb = NULL,
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    .application_message.message_cplt_cb = NULL,
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    .application_message.ul_payload = NULL,
#endif /* !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0) */
#ifndef SIGFOX_EP_UL_PAYLOAD_SIZE
    .application_message.ul_payload_size_bytes = 0,
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#ifdef SIGFOX_EP_BIDIRECTIONAL
    .application_message.bidirectional_flag = SIGFOX_FALSE,
#ifndef SIGFOX_EP_T_CONF_MS
    .application_message.t_conf_ms = 1500,
#endif /* SIGFOX_EP_T_CONF_MS */
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
// Init the control message configuration
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    .control_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS,
#endif /* SIGFOX_EP_UL_BIT_RATE_BPS */
#ifndef SIGFOX_EP_TX_POWER_DBM_EIRP
    .control_message.common_parameters.tx_power_dbm_eirp = 14,
#endif /* SIGFOX_EP_TX_POWER_DBM_EIRP */
#ifndef SIGFOX_EP_SINGLE_FRAME
    .control_message.common_parameters.number_of_frames = 3,
#ifndef SIGFOX_EP_T_IFU_MS
    .control_message.common_parameters.t_ifu_ms = 500,
#endif /* SIGFOX_EP_T_IFU_MS */
#endif /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    .control_message.common_parameters.ep_key_type = SIGFOX_EP_KEY_PRIVATE,
#endif /* SIGFOX_EP_PUBLIC_KEY_CAPABLE */
#endif /* !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE) */
    .control_message.type = SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE,
#ifdef SIGFOX_EP_ASYNCHRONOUS
    .control_message.uplink_cplt_cb = NULL,
    .control_message.message_cplt_cb = NULL,
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#endif /* SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE */
};

/*** CLI local functions ***/

/*******************************************************************/
static void _at_process_callback(void) {
    // Set flag.
    cli_ctx.flags.field.at_process = 1;
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static void _print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm) {
    // Local variables.
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    uint8_t idx = 0;
    // Check parameter.
    if (dl_payload == NULL) {
        goto errors;
    }
    // DL payload.
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, " dl_payload=");
    for (idx = 0; idx < dl_payload_size; idx++) {
        sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s%02X", reply, dl_payload[idx]);
    }
    // RSSI.
    sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s,rssi=", reply);
    sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s%d", reply, rssi_dbm);
    // Print data.
    AT_send_reply(NULL, reply);
    return;
errors:
    return;
}
#endif /* SIGFOX_EP_BIDIRECTIONAL */

/*******************************************************************/
static const char *_cli_error_enum_to_str_callback(unsigned int error_code) {
    return CLI_STATUS_STR[error_code];
}

/*******************************************************************/
static AT_status_t _print_modem_information(uint8_t index, int32_t *error_code) {
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */

    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    uint8_t mcu_id[MCU_CHIP_ID_SIZE_BYTES] = {0x00};
    uint8_t idx = 0;
    // Reset error code.
    *error_code = CLI_SUCCESS;
    // Check index.
    switch (index) {
    case CLI_INFORMATION_INDEX_CHIP_ID:
        snprintf(reply, CLI_REPLY_BUFFER_SIZE, "Chip_id:");
        // Read ID.
        mcal_status = MCU_get_chip_id(mcu_id);
        MCAL_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_AT);
        // Loop on DIE ID bytes.
        for (idx = 0; idx < MCU_CHIP_ID_SIZE_BYTES; idx++) {
            sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s%02X", reply, mcu_id[idx]);
        }
        break;
    case CLI_INFORMATION_INDEX_SIGFOX_EP_ID:
#ifdef SIGFOX_EP_VERBOSE
        snprintf(reply, CLI_REPLY_BUFFER_SIZE, "Sigfox_ep_id:");
#ifdef SIGFOX_EP_ERROR_CODES
        sigfox_ep_api_status = SIGFOX_EP_API_open(&cli_ctx.lib_config);
        SIGFOX_EP_API_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_LIB);
        sigfox_ep_api_status = SIGFOX_EP_API_get_ep_id(mcu_id, SIGFOX_EP_ID_SIZE_BYTES);
        SIGFOX_EP_API_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_LIB);
        sigfox_ep_api_status = SIGFOX_EP_API_close();
        SIGFOX_EP_API_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_LIB);
#else
        SIGFOX_EP_API_open(&cli_ctx.lib_config);
        SIGFOX_EP_API_get_ep_id(mcu_id, SIGFOX_EP_ID_SIZE_BYTES);
        SIGFOX_EP_API_close();
#endif /* SIGFOX_EP_ERROR_CODES */
        // Loop on ID bytes.
        for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
            // Read byte.
            sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s%02X", reply, mcu_id[idx]);
        }
#endif /* SIGFOX_EP_VERBOSE */
        break;
    case CLI_INFORMATION_INDEX_FIRMWARE_VERSION:
        snprintf(reply, CLI_REPLY_BUFFER_SIZE, "Firmware:%s", CLI_FIRMWARE_VERSION);
        break;
    default:
        *error_code = 0x00; // Parameter position
        status = AT_ERROR_EXTERNAL_COMMAND_BAD_PARAMETER_VALUE;
        goto errors;
    }
    // Print data.
    status = AT_send_reply(&CLI_COMMAND_INFO, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_MCAL);
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _info_execution_callback(int32_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    uint8_t idx = 0;
    // Print all informations.
    for (idx = 0; idx < CLI_INFORMATION_INDEX_LAST; idx++) {
        status = _print_modem_information(idx, error_code);
        if (status != AT_SUCCESS) {
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _info_write_callback(uint32_t argc, char *argv[], int32_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    int tmp_int = 0;
    // Check number of arguments.
    AT_command_check_and_exit_param_number_error(1);
    // Parse parameter.
    if (sscanf((argv[0]), "%d", &tmp_int) != 1) {
        status = AT_ERROR_EXTERNAL_COMMAND_BAD_PARAMETER_PARSING;
        goto errors;
    }
    status = _print_modem_information((uint8_t) tmp_int, error_code);
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _rc_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    uint8_t rc_index = 0;
    uint8_t rc_valid = 0;
    // Reset error code.
    *error_code = CLI_SUCCESS;
    AT_command_check_and_exit_param_number_error(1);
    for (rc_index = 0; rc_index < (sizeof(CLI_SIGFOX_RC) / sizeof(CLI_sigfox_rc_t)); rc_index++) {
        if (strcmp(argv[0], CLI_SIGFOX_RC[rc_index].name) == 0) {
            rc_valid = 1;
            break;
        }
    }
    AT_command_check_and_exit_param_value_error(rc_valid == 0, 0);
    if (CLI_SIGFOX_RC[rc_index].ptr == NULL) {
        AT_command_exit_core_error(CLI_ERROR_RC_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
    }
    cli_ctx.lib_config.rc = CLI_SIGFOX_RC[rc_index].ptr;

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
#ifndef SIGFOX_EP_TX_POWER_DBM_EIRP
    cli_ctx.application_message.common_parameters.tx_power_dbm_eirp = CLI_SIGFOX_RC[rc_index].ptr->tx_power_dbm_eirp_max - DEFAULT_ANTENNA_GAIN_DBI;
#endif /* SIGFOX_EP_TX_POWER_DBM_EIRP */
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    cli_ctx.application_message.common_parameters.ul_bit_rate = CLI_SIGFOX_RC[rc_index].default_sigfox_ul_bit_rate;
#endif /* SIGFOX_EP_UL_BIT_RATE_BPS */
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
#ifndef SIGFOX_EP_TX_POWER_DBM_EIRP
    cli_ctx.control_message.common_parameters.tx_power_dbm_eirp = CLI_SIGFOX_RC[rc_index].ptr->tx_power_dbm_eirp_max - DEFAULT_ANTENNA_GAIN_DBI;
#endif /* SIGFOX_EP_TX_POWER_DBM_EIRP */
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    cli_ctx.control_message.common_parameters.ul_bit_rate = CLI_SIGFOX_RC[rc_index].default_sigfox_ul_bit_rate;
#endif /* SIGFOX_EP_UL_BIT_RATE_BPS */
#endif /* SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE */
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _rc_read_callback(CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    uint8_t rc_index = 0;
    // Reset error code.
    *error_code = CLI_SUCCESS;
    // Print RC.
    for (rc_index = 0; rc_index < sizeof(CLI_SIGFOX_RC) / sizeof(CLI_sigfox_rc_t); rc_index++) {
        if (cli_ctx.lib_config.rc == CLI_SIGFOX_RC[rc_index].ptr) {
            snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s", CLI_SIGFOX_RC[rc_index].name);
            at_status = AT_send_reply(NULL, reply);
            AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_AT);
            break;
        }
    }
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _ul_br_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    int ulbr_value = 0;
    // Reset error code.
    *error_code = CLI_SUCCESS;
    AT_command_check_and_exit_param_number_error(1);
    AT_command_check_and_exit_param_parser_error(argv[0] == NULL, 0);
    AT_command_check_and_exit_param_parser_error(sscanf((argv[0]), "%d", &ulbr_value) != 1, 0);
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    if (ulbr_value == 600) {
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
        cli_ctx.application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
        cli_ctx.control_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
#endif /* SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE */
    } else if (ulbr_value == 100) {
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
        cli_ctx.application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
        cli_ctx.control_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#endif /* SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE */
    } else {
        AT_command_exit_param_value_error(0);
    }
#else
    AT_command_exit_core_error(CLI_ERROR_UL_BR_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_UL_BIT_RATE_BPS */

    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _ul_br_read_callback(CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    // Reset error code.
    *error_code = CLI_SUCCESS;
    // Print ULBR.
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    if (cli_ctx.application_message.common_parameters.ul_bit_rate == SIGFOX_UL_BIT_RATE_100BPS) {
#elif defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    if (cli_ctx.control_message.common_parameters.ul_bit_rate == SIGFOX_UL_BIT_RATE_100BPS) {
#endif /* SIGFOX_EP_APPLICATION_MESSAGES*/
        snprintf(reply, CLI_REPLY_BUFFER_SIZE, "100");
    } else {
        snprintf(reply, CLI_REPLY_BUFFER_SIZE, "600");
    }
#else
    sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", SIGFOX_EP_UL_BIT_RATE_BPS);
#endif /* SIGFOX_EP_UL_BIT_RATE_BPS */
    at_status = AT_send_reply(NULL, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_AT);
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _n_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    int n_value = 0;
    // Reset error code.
    *error_code = CLI_SUCCESS;
    AT_command_check_and_exit_param_number_error(1);
    AT_command_check_and_exit_param_parser_error(argv[0] == NULL, 0);
    AT_command_check_and_exit_param_parser_error(sscanf((argv[0]), "%d", &n_value) != 1, 0);
#ifndef SIGFOX_EP_SINGLE_FRAME
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    cli_ctx.application_message.common_parameters.number_of_frames = n_value;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    cli_ctx.control_message.common_parameters.number_of_frames = n_value;
#endif /* SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE */
#else
    AT_command_exit_core_error(CLI_ERROR_N_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_SINGLE_FRAME */
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _n_read_callback(CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    // Reset error code.
    *error_code = CLI_SUCCESS;
    // Print N.
#ifndef SIGFOX_EP_SINGLE_FRAME
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", cli_ctx.application_message.common_parameters.number_of_frames);
#elif defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", cli_ctx.control_message.common_parameters.number_of_frames);
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#else
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "1");
#endif /* SIGFOX_EP_SINGLE_FRAME */
    at_status = AT_send_reply(NULL, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_AT);
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _t_ifu_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    int t_ifu_value = 0;
    // Reset error code.
    *error_code = CLI_SUCCESS;
    AT_command_check_and_exit_param_number_error(1);
    AT_command_check_and_exit_param_parser_error(argv[0] == NULL, 0);
    AT_command_check_and_exit_param_parser_error(sscanf((argv[0]), "%d", &t_ifu_value) != 1, 0);
#if !(defined SIGFOX_EP_T_IFU_MS) && !(defined SIGFOX_EP_SINGLE_FRAME)
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    cli_ctx.application_message.common_parameters.t_ifu_ms = t_ifu_value;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    cli_ctx.control_message.common_parameters.t_ifu_ms = t_ifu_value;
#endif /* SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE */
#else
    AT_command_exit_core_error(CLI_ERROR_T_IFU_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* !(defined SIGFOX_EP_T_IFU_MS) && !(defined SIGFOX_EP_SINGLE_FRAME) */

    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _t_ifu_read_callback(CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    // Reset error code.
    *error_code = CLI_SUCCESS;
    // Print T_IFU.
#ifndef SIGFOX_EP_SINGLE_FRAME
#ifndef SIGFOX_EP_T_IFU_MS
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", cli_ctx.application_message.common_parameters.t_ifu_ms);
#elif defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", cli_ctx.control_message.common_parameters.t_ifu_ms);
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#else
    sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", SIGFOX_EP_T_IFU_MS);
#endif /* SIGFOX_EP_T_IFU_MS */
#else
    AT_command_exit_core_error(CLI_ERROR_T_IFU_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_SINGLE_FRAME */
    at_status = AT_send_reply(NULL, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_AT);
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _key_type_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    uint8_t key_type_index = 0;
    uint8_t key_type_valid = 0;
#endif /* SIGFOX_EP_PUBLIC_KEY_CAPABLE */
    // Reset error code.
    *error_code = CLI_SUCCESS;
    AT_command_check_and_exit_param_number_error(1);
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    for (key_type_index = 0; key_type_index < (sizeof(CLI_SIGFOX_EP_KEY_TYPE) / sizeof(CLI_ep_key_type_t)); key_type_index++) {
        if (strcmp(argv[0], CLI_SIGFOX_EP_KEY_TYPE[key_type_index].name) == 0) {
            key_type_valid = 1;
            break;
        }
    }
    AT_command_check_and_exit_param_value_error(key_type_valid == 0, 0);
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    cli_ctx.application_message.common_parameters.ep_key_type = CLI_SIGFOX_EP_KEY_TYPE[key_type_index].ep_key_type;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    cli_ctx.control_message.common_parameters.ep_key_type = CLI_SIGFOX_EP_KEY_TYPE[key_type_index].ep_key_type;
#endif /* SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE */
#else
    AT_command_exit_core_error(CLI_ERROR_KEY_TYPE_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_PUBLIC_KEY_CAPABLE */

    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _key_type_read_callback(CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    // Reset error code.
    *error_code = CLI_SUCCESS;
    // Print Key Type.
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    if (cli_ctx.application_message.common_parameters.ep_key_type == SIGFOX_EP_KEY_PRIVATE) {
#elif defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    if (cli_ctx.control_message.common_parameters.ep_key_type == SIGFOX_EP_KEY_PRIVATE) {
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
        snprintf(reply, CLI_REPLY_BUFFER_SIZE, "PRIVATE");
    } else {
        snprintf(reply, CLI_REPLY_BUFFER_SIZE, "PUBLIC");
    }
#else
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "PRIVATE");
#endif /* SIGFOX_EP_PUBLIC_KEY_CAPABLE */
    at_status = AT_send_reply(NULL, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_AT);
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _t_conf_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    int t_conf_value = 0;
    // Reset error code.
    *error_code = CLI_SUCCESS;
    AT_command_check_and_exit_param_number_error(1);
    AT_command_check_and_exit_param_parser_error(argv[0] == NULL, 0);
    AT_command_check_and_exit_param_parser_error(sscanf((argv[0]), "%d", &t_conf_value) != 1, 0);
#if (defined SIGFOX_EP_BIDIRECTIONAL) && !(defined SIGFOX_EP_T_CONF_MS)
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    cli_ctx.application_message.t_conf_ms = t_conf_value;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#else
    AT_command_exit_core_error(CLI_ERROR_T_CONF_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _t_conf_read_callback(CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    // Reset error code.
    *error_code = CLI_SUCCESS;
    // Print T_CONF.
#ifdef SIGFOX_EP_BIDIRECTIONAL
#ifndef SIGFOX_EP_T_CONF_MS
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", cli_ctx.application_message.t_conf_ms);
#elif defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", cli_ctx.control_message.t_conf_ms);
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#else
    sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d", SIGFOX_EP_T_CONF_MS);
#endif /* SIGFOX_EP_T_CONF_MS */
#else
    AT_command_exit_core_error(CLI_ERROR_T_CONF_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    at_status = AT_send_reply(NULL, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_AT);
    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _errors_read_callback(CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
#ifdef SIGFOX_EP_ERROR_STACK
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_ERROR_t sigfox_ep_api_error;
    uint8_t idx = 0;
#endif /* SIGFOX_EP_ERROR_STACK */
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    // Reset error code.
    *error_code = CLI_SUCCESS;
    // Print Stack.
#ifdef SIGFOX_EP_ERROR_STACK
    for (idx = 0; idx < SIGFOX_EP_ERROR_STACK; idx++) {
        sigfox_ep_api_status = SIGFOX_EP_API_unstack_error(&sigfox_ep_api_error);
        SIGFOX_EP_API_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_LIB);
        if (sigfox_ep_api_error.code == SIGFOX_EP_API_SUCCESS) {
            break;
        }
        sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s%d,%ld ", reply, sigfox_ep_api_error.source, sigfox_ep_api_error.code);
    }
#else
    AT_command_exit_core_error(CLI_ERROR_ERROR_STACK_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_ERROR_STACK */
    at_status = AT_send_reply(NULL, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_AT);

    return AT_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _msg_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    AT_status_t at_status = AT_SUCCESS;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    SIGFOX_EP_API_message_status_t message_status;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
    int tmp_int;
    uint8_t idx = 0;
    SIGFOX_UNUSED(tmp_int);
    SIGFOX_UNUSED(idx);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_u8 dl_payload[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm = 0;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#if (!(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)) && (defined SIGFOX_EP_APPLICATION_MESSAGES)
    sfx_u8 ul_payload[SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES] = {0x00};
#endif /* (!(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)) && (defined SIGFOX_EP_APPLICATION_MESSAGES) */
    // Reset error code.
    *error_code = CLI_SUCCESS;
    AT_command_check_and_exit_param_number_error(2);
    // Parse payload data from the command
    AT_command_check_and_exit_param_parser_error(argv[0] == NULL, 0);
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    if (strcmp(argv[0], "NULL") == 0) {
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 0)
        cli_ctx.application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY;
#else
        AT_command_exit_core_error(CLI_ERROR_UL_PAYLOAD_EMPTY_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#else
        cli_ctx.application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY;
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
    } else {
        if (strlen(argv[0]) == 1) {
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 0)
            if (*argv[0] == '0') {
                cli_ctx.application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0;
            } else if (*argv[0] == '1') {
                cli_ctx.application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BIT1;
            } else {
                AT_command_exit_param_value_error(0);
            }
#else
            AT_command_exit_core_error(CLI_ERROR_UL_PAYLOAD_BIT_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 0) */
        } else {
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 0)
            AT_command_exit_core_error(CLI_ERROR_UL_PAYLOAD_BYTES_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#else
            AT_command_check_and_exit_param_parser_error(strlen(argv[0]) % 2 != 0, 0);
            AT_command_check_and_exit_param_value_error((strlen(argv[0]) / 2) > SIGFOX_EP_UL_PAYLOAD_SIZE, 0);
            cli_ctx.application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
            cli_ctx.application_message.ul_payload = ul_payload;
            for (idx = 0; idx < SIGFOX_EP_UL_PAYLOAD_SIZE; idx++) {
                AT_command_check_and_exit_param_parser_error(sscanf((argv[0] + (idx * 2)), "%2X", &tmp_int) != 1, 0);
                ul_payload[idx] = (uint8_t) tmp_int;
            }
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#else
            AT_command_check_and_exit_param_parser_error(strlen(argv[0]) % 2 != 0, 0);
            AT_command_check_and_exit_param_value_error((strlen(argv[0]) / 2) > SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES, 0);
            cli_ctx.application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
            cli_ctx.application_message.ul_payload_size_bytes = strlen(argv[0]) / 2;
            cli_ctx.application_message.ul_payload = ul_payload;
            for (idx = 0; idx < cli_ctx.application_message.ul_payload_size_bytes; idx++) {
                AT_command_check_and_exit_param_parser_error(sscanf((argv[0] + (idx * 2)), "%2X", &tmp_int) != 1, 0);
                ul_payload[idx] = (uint8_t) tmp_int;
            }
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
        }
    }
    // Parse bidir flag from the command
    AT_command_check_and_exit_param_parser_error(argv[1] == NULL, 1);
    AT_command_check_and_exit_param_parser_error(sscanf((argv[1]), "%d", &tmp_int) != 1, 1);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    cli_ctx.application_message.bidirectional_flag = (sfx_bool) tmp_int;
#else
    if (tmp_int == 1) {
        AT_command_exit_core_error(CLI_ERROR_BIDIR_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
    }
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    // Open the library
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_api_status = SIGFOX_EP_API_open(&cli_ctx.lib_config);
    SIGFOX_EP_API_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_LIB);
#else
    SIGFOX_EP_API_open(&cli_ctx.lib_config);
#endif /* SIGFOX_EP_ERROR_CODES */
    // Send the message
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(&cli_ctx.application_message);
    SIGFOX_EP_API_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_LIB);
#else
    SIGFOX_EP_API_send_application_message(&cli_ctx.application_message);
#endif /* SIGFOX_EP_ERROR_CODES */
    // Read message status.
    message_status = SIGFOX_EP_API_get_message_status();
    // Print message status.
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d,%d,%d,%d,%d,%d,%d", message_status.field.ul_frame_1, message_status.field.ul_frame_2, message_status.field.ul_frame_3, message_status.field.dl_frame, message_status.field.dl_conf_frame, message_status.field.network_error, message_status.field.execution_error);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Print DL data if needed.
    if (message_status.field.dl_frame != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        sigfox_ep_api_status = SIGFOX_EP_API_get_dl_payload(dl_payload, SIGFOX_DL_PAYLOAD_SIZE_BYTES, &dl_rssi_dbm);
        SIGFOX_EP_API_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_LIB);
#else
        SIGFOX_EP_API_get_dl_payload(dl_payload, SIGFOX_DL_PAYLOAD_SIZE_BYTES, &dl_rssi_dbm);
#endif /* SIGFOX_EP_ERROR_CODES */
        for (idx = 0; idx < SIGFOX_DL_PAYLOAD_SIZE_BYTES; idx++) {
            sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s%02X", reply, dl_payload[idx]);
        }
        sniprintf(reply, CLI_REPLY_BUFFER_SIZE, "%s,%d", reply, dl_rssi_dbm);
    }
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    // Close the library
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    SIGFOX_EP_API_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_LIB);
#else
    SIGFOX_EP_API_close();
#endif /* SIGFOX_EP_ERROR_CODES */
    at_status = AT_send_reply(&CLI_COMMAND_MSG, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_MCAL);
    return AT_SUCCESS;
#else
    AT_command_exit_core_error(CLI_ERROR_APP_MESSAGE_BYTES_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
errors:
    // Force to close Library
    SIGFOX_EP_API_close();
    return status;
}

/*******************************************************************/
static AT_status_t _rfp_write_callback(uint32_t argc, char *argv[], CLI_status_t *error_code) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_ADDON_RFP_API_status_t sigfox_ep_addon_rfp_status;
#endif
    uint8_t rfp_index = 0;
    uint8_t rfp_valid = 0;
    char reply[CLI_REPLY_BUFFER_SIZE] = {0x00};
    SIGFOX_EP_ADDON_RFP_API_config_t sigfox_ep_addon_rfp_api_config;
    SIGFOX_EP_ADDON_RFP_API_test_mode_t sigfox_ep_addon_rfp_test_mode;
    SIGFOX_EP_ADDON_RFP_API_progress_status_t sigfox_ep_addon_rfp_progress_status;
    // Reset error code.
    *error_code = CLI_SUCCESS;
    AT_command_check_and_exit_param_number_error(1);
    for (rfp_index = 0; rfp_index < (sizeof(CLI_SIGFOX_EP_ADDON_RFP_TEST_MODE) / sizeof(CLI_sigfox_ep_addon_rfp_test_mode_t)); rfp_index++) {
        if (strcmp(argv[0], CLI_SIGFOX_EP_ADDON_RFP_TEST_MODE[rfp_index].letter) == 0) {
            rfp_valid = 1;
            break;
        }
    }
    AT_command_check_and_exit_param_value_error(rfp_valid == 0, 0);
    if (CLI_SIGFOX_EP_ADDON_RFP_TEST_MODE[rfp_index].test_mode_reference == 14) {
        AT_command_exit_core_error(CLI_ERROR_RFP_TEST_MODE_NOT_SUPPORTED_IN_THIS_CONFIGURATION);
    }
    sigfox_ep_addon_rfp_api_config.rc = cli_ctx.lib_config.rc;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    sigfox_ep_addon_rfp_api_config.process_cb = NULL;
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#ifndef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
    sigfox_ep_addon_rfp_api_config.message_counter_rollover = cli_ctx.lib_config.message_counter_rollover;
#endif /* SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER */
    sigfox_ep_addon_rfp_test_mode.test_mode_reference = CLI_SIGFOX_EP_ADDON_RFP_TEST_MODE[rfp_index].test_mode_reference;
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    sigfox_ep_addon_rfp_test_mode.ul_bit_rate = cli_ctx.application_message.common_parameters.ul_bit_rate;
#elif defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    sigfox_ep_addon_rfp_test_mode.ul_bit_rate = cli_ctx.control_message.common_parameters.ul_bit_rate;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#endif /* SIGFOX_EP_UL_BIT_RATE_BPS */
#ifndef SIGFOX_EP_TX_POWER_DBM_EIRP
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    sigfox_ep_addon_rfp_test_mode.tx_power_dbm_eirp = cli_ctx.application_message.common_parameters.tx_power_dbm_eirp;
#elif defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    sigfox_ep_addon_rfp_test_mode.tx_power_dbm_eirp = cli_ctx.control_message.common_parameters.tx_power_dbm_eirp;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#endif /* SIGFOX_EP_TX_POWER_DBM_EIRP */
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sigfox_ep_addon_rfp_test_mode.downlink_cplt_cb = &_print_dl_payload;
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
    sigfox_ep_addon_rfp_test_mode.test_mode_cplt_cb = NULL;
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_open(&sigfox_ep_addon_rfp_api_config);
    if (sigfox_ep_addon_rfp_status != SIGFOX_EP_ADDON_RFP_API_SUCCESS) {
        *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_ADDON_RFP;
        status = AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR;
        goto errors;
    }
#else
    SIGFOX_EP_ADDON_RFP_API_open(&sigfox_ep_addon_rfp_api_config);
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_test_mode(&sigfox_ep_addon_rfp_test_mode);
    if (sigfox_ep_addon_rfp_status != SIGFOX_EP_ADDON_RFP_API_SUCCESS) {
        *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_ADDON_RFP;
        status = AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR;
        goto errors;
    }
#else
    SIGFOX_EP_ADDON_RFP_API_test_mode(&sigfox_ep_addon_rfp_test_mode);
#endif /* SIGFOX_EP_ERROR_CODES */
    sigfox_ep_addon_rfp_progress_status = SIGFOX_EP_ADDON_RFP_API_get_test_mode_progress_status();
    snprintf(reply, CLI_REPLY_BUFFER_SIZE, "%d,%d", sigfox_ep_addon_rfp_progress_status.status.error, sigfox_ep_addon_rfp_progress_status.progress);
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_addon_rfp_status = SIGFOX_EP_ADDON_RFP_API_close();
    if (sigfox_ep_addon_rfp_status != SIGFOX_EP_ADDON_RFP_API_SUCCESS) {
        *error_code = CLI_ERROR_DRIVER_SIGFOX_EP_ADDON_RFP;
        status = AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR;
        goto errors;
    }
#else
    SIGFOX_EP_ADDON_RFP_API_close();
#endif /* SIGFOX_EP_ERROR_CODES */
    at_status = AT_send_reply(&CLI_COMMAND_RFP, reply);
    AT_check_status(AT_ERROR_EXTERNAL_COMMAND_CORE_ERROR; *error_code = CLI_ERROR_DRIVER_MCAL);
    return AT_SUCCESS;
errors:
    SIGFOX_EP_ADDON_RFP_API_close();
    return status;
}

/*** CLI functions ***/

/*******************************************************************/
CLI_status_t CLI_init(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    AT_config_t at_config;
    // Init AT configuration.
    at_config.process_callback = (AT_process_cb_t) &_at_process_callback;
    at_config.default_echo_flag = 0;
    at_config.default_quiet_flag = 0;
    at_config.default_verbose_flag = 1;
    // Init AT manager.
    at_status = AT_init(&at_config);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    // Registers always available commands.
    at_status = AT_register_command(&CLI_COMMAND_INFO);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_RC);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_UL_BR);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_N);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_T_IFU);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_KEY_TYPE);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_T_CONF);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_ERRORS);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_MSG);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    at_status = AT_register_command(&CLI_COMMAND_RFP);
    AT_check_status(CLI_ERROR_DRIVER_AT);
    return CLI_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
CLI_status_t CLI_de_init(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    // Init AT manager.
    at_status = AT_de_init();
    AT_check_status(CLI_ERROR_DRIVER_AT);
    return CLI_SUCCESS;
errors:
    return status;
}

/*******************************************************************/
CLI_status_t CLI_process(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    AT_status_t at_status = AT_SUCCESS;
    // Check flags.
    if (cli_ctx.flags.field.at_process != 0) {
        // Clear flag.
        cli_ctx.flags.field.at_process = 0;
        // Process AT driver.
        at_status = AT_process();
        AT_check_status(CLI_ERROR_DRIVER_AT);
    }
errors:
    return status;
}
