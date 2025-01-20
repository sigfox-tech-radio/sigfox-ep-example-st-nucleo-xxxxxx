 ###########################################################################
 #                   _____ _____ _____ ______ ______   __                  #
 #                  / ____|_   _/ ____|  ____/ __ \ \ / /                  #
 #                 | (___   | || |  __| |__ | |  | \ V /                   #
 #                  \___ \  | || | |_ |  __|| |  | |> <                    #
 #                  ____) |_| || |__| | |   | |__| / . \                   #
 #                 |_____/|_____\_____|_|    \____/_/ \_\                  #
 #                               ___   _____                               #
 #                              / _ \ / ____|                              #
 #                             | | | | |  __                               #
 #                             | | | | | |_ |                              #
 #                             | |_| | |__| |                              #
 #                              \___/ \_____|                              #
 #  _______ ______ _____ _    _ _   _  ____  _      ____   _______     __  #
 # |__   __|  ____/ ____| |  | | \ | |/ __ \| |    / __ \ / ____\ \   / /  #
 #    | |  | |__ | |    | |__| |  \| | |  | | |   | |  | | |  __ \ \_/ /   #
 #    | |  |  __|| |    |  __  | . ` | |  | | |   | |  | | | |_ | \   /    #
 #    | |  | |___| |____| |  | | |\  | |__| | |___| |__| | |__| |  | |     #
 #    |_|  |______\_____|_|  |_|_| \_|\____/|______\____/ \_____|  |_|     #
 #          ______     __  _    _ _   _          ____ _____ ______         #
 #         |  _ \ \   / / | |  | | \ | |   /\   |  _ \_   _|___  /         #
 #         | |_) \ \_/ /  | |  | |  \| |  /  \  | |_) || |    / /          #
 #         |  _ < \   /   | |  | | . ` | / /\ \ |  _ < | |   / /           #
 #         | |_) | | |    | |__| | |\  |/ ____ \| |_) || |_ / /__          #
 #         |____/  |_|     \____/|_| \_/_/    \_\____/_____/_____|         #
 #                                                                         #
 ###########################################################################
 #
 # Copyright (c) 2022, UnaBiz SAS
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #
 #  1 Redistributions of source code must retain the above copyright notice,
 #    this list of conditions and the following disclaimer.
 #  2 Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 #    used to endorse or promote products derived from this software without
 #    specific prior written permission.
 #
 # NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 # THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 # CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 # PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 # BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 # IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 # POSSIBILITY OF SUCH DAMAGE.
 #
 ###########################################################################

cmake_minimum_required(VERSION 3.21)
project(sigfox-ep-example-st-nucleo-xxxxxx)

#Configuration variables 
set(RADIO_SHIELD  lr1110mb1dis  CACHE STRING "Configure MCU board (default: none)")
set_property(CACHE RADIO_SHIELD PROPERTY STRINGS none sx1261mb1bas sx1261mb2bas lr1110mb1dis lr1110mb1djs lr1121mb1dis steval-fki868v2)
set(MCU_BOARD  NucleoL053R8  CACHE STRING "Configure MCU board (default: NucleoL053R8)")
set_property(CACHE MCU_BOARD PROPERTY STRINGS NucleoL053R8 NucleoL152RE NucleoWL33CC1)

#MCU configuration
if(MCU_BOARD STREQUAL NucleoL053R8)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus")
    set(MCU stm32l053r8)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/drivers/cmsis EXCLUDE_FROM_ALL)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/drivers/peripherals EXCLUDE_FROM_ALL)
elseif(MCU_BOARD STREQUAL NucleoL152RE)
    message( SEND_ERROR "Not implemented" )
elseif(MCU_BOARD STREQUAL NucleoWL33CC1)
    message( SEND_ERROR "Not implemented" )
else()
    message( SEND_ERROR "MCU_BOARD must be configured" )
endif()

#Radio shield configuration (not mandatory if mcu is an SoC with a radio)
if(RADIO_SHIELD STREQUAL lr1110mb1dis)
    set(RADIO_DRIVER lr11xx_driver)
    set(SIGFOX_RF_API lr11xx_rf_api_obj)
elseif(RADIO_SHIELD STREQUAL lr1110mb1djs)
    set(RADIO_DRIVER lr11xx_driver)
    set(SIGFOX_RF_API lr11xx_rf_api_obj)
elseif(RADIO_SHIELD STREQUAL lr1121mb1dis)
    set(RADIO_DRIVER lr11xx_driver)
    set(SIGFOX_RF_API lr11xx_rf_api_obj)
elseif(RADIO_SHIELD STREQUAL sx1261mb1bas)
    set(RADIO_DRIVER sx126x_driver)
    set(SIGFOX_RF_API sx126x_rf_api_obj)
elseif(RADIO_SHIELD STREQUAL sx1261mb2bas)
    set(RADIO_DRIVER sx126x_driver)
    set(SIGFOX_RF_API sx126x_rf_api_obj)
elseif(RADIO_SHIELD STREQUAL steval-fki868v2)
    set(RADIO_DRIVER s2lp_driver)
    set(SIGFOX_RF_API s2lp_rf_api_obj)
else()
    message( SEND_ERROR "RADIO_SHIELD must be configured" )
endif()

#Generic variable and subdirectories nedded for all projects
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/drivers/components EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/drivers/shields EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/drivers/utils EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/middleware/at EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/middleware/sigfox EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/application/button)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/application/modem)
