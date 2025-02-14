################################################################################
#                      _____ _____ _____ ______ ______   __                    #
#                     / ____|_   _/ ____|  ____/ __ \ \ / /                    #
#                    | (___   | || |  __| |__ | |  | \ V /                     #
#                     \___ \  | || | |_ |  __|| |  | |> <                      #
#                     ____) |_| || |__| | |   | |__| / . \                     #
#                    |_____/|_____\_____|_|    \____/_/ \_\                    #
#                                  ___   _____                                 #
#                                 / _ \ / ____|                                #
#                                | | | | |  __                                 #
#                                | | | | | |_ |                                #
#                                | |_| | |__| |                                #
#                                 \___/ \_____|                                #
#     _______ ______ _____ _    _ _   _  ____  _      ____   _______     __    #
#    |__   __|  ____/ ____| |  | | \ | |/ __ \| |    / __ \ / ____\ \   / /    #
#       | |  | |__ | |    | |__| |  \| | |  | | |   | |  | | |  __ \ \_/ /     #
#       | |  |  __|| |    |  __  | . ` | |  | | |   | |  | | | |_ | \   /      #
#       | |  | |___| |____| |  | | |\  | |__| | |___| |__| | |__| |  | |       #
#       |_|  |______\_____|_|  |_|_| \_|\____/|______\____/ \_____|  |_|       #
#             ______     __  _    _ _   _          ____ _____ ______           #
#            |  _ \ \   / / | |  | | \ | |   /\   |  _ \_   _|___  /           #
#            | |_) \ \_/ /  | |  | |  \| |  /  \  | |_) || |    / /            #
#            |  _ < \   /   | |  | | . ` | / /\ \ |  _ < | |   / /             #
#            | |_) | | |    | |__| | |\  |/ ____ \| |_) || |_ / /__            #
#            |____/  |_|     \____/|_| \_/_/    \_\____/_____/_____|           #
#                                                                              #
################################################################################
#
# Copyright (c) 2024, UnaBiz SAS
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
################################################################################

cmake_minimum_required(VERSION 3.21)
project(shield)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL)
if(RADIO_DRIVER STREQUAL lr11xx_driver)
  # Add shield specific defines and source files
  if (RADIO_SHIELD STREQUAL lr1110mb1dis)
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_shields/src/lr1110mb1dis_hw_api.c
  )
  elseif (RADIO_SHIELD STREQUAL lr1110mb1djs)
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_shields/src/lr1110mb1djs_hw_api.c
  )
  elseif (RADIO_SHIELD STREQUAL lr1121mb1dis)
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_shields/src/lr1121mb1dis_hw_api.c
  )
  endif()
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_shields/src/lr11xx_hal.c
  )
  target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_shields/inc/
    $<TARGET_PROPERTY:lr11xx_rf_api_obj,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:components,INTERFACE_INCLUDE_DIRECTORIES>
  )
  target_compile_definitions(${PROJECT_NAME} 
    PUBLIC 
    LR11XX_DISABLE_WARNINGS
    $<TARGET_PROPERTY:lr11xx_rf_api_obj,INTERFACE_COMPILE_DEFINITIONS>
    $<TARGET_PROPERTY:components,INTERFACE_COMPILE_DEFINITIONS>
  )
elseif(RADIO_DRIVER STREQUAL sx126x_driver)
  # Add shield specific defines and source files
  if (RADIO_SHIELD STREQUAL sx1261mb1bas)
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sx126x_shields/src/sx1261mb1bas_hw_api.c
  )
  elseif(RADIO_SHIELD STREQUAL sx1261mb2bas)
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sx126x_shields/src/sx1261mb2bas_hw_api.c
  )
  endif()
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sx126x_shields/src/sx126x_hal.c
  )
  target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/sx126x_shields/inc/
    $<TARGET_PROPERTY:sx126x_rf_api_obj,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:components,INTERFACE_INCLUDE_DIRECTORIES>
  )
  target_compile_definitions(${PROJECT_NAME} 
    PUBLIC 
    $<TARGET_PROPERTY:sx126x_rf_api_obj,INTERFACE_COMPILE_DEFINITIONS>
    $<TARGET_PROPERTY:components,INTERFACE_COMPILE_DEFINITIONS>
  )

elseif(RADIO_DRIVER STREQUAL s2lp_driver)
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/s2lp_shields/src/S2LP_CORE_SPI.c
    ${CMAKE_CURRENT_SOURCE_DIR}/s2lp_shields/src/steval_fki868v2_hw_api.c
  )
  target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/s2lp_shields/inc/
    $<TARGET_PROPERTY:s2lp_rf_api_obj,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:components,INTERFACE_INCLUDE_DIRECTORIES>
  )
  target_compile_definitions(${PROJECT_NAME} 
    PUBLIC 
    LR11XX_DISABLE_WARNINGS
    $<TARGET_PROPERTY:s2lp_rf_api_obj,INTERFACE_COMPILE_DEFINITIONS>
    $<TARGET_PROPERTY:components,INTERFACE_COMPILE_DEFINITIONS>
  )
elseif(RADIO_DRIVER STREQUAL stm32wl3x_driver)
  target_sources(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x_shields/src/nucleo_wl33cc1_hw_api.c
    PUBLIC
    $<TARGET_OBJECTS:peripherials>
  )
  target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<TARGET_PROPERTY:stm32wl3x_rf_api_obj,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:components,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:cmsis,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:peripherials,INTERFACE_INCLUDE_DIRECTORIES>
  )
  target_compile_definitions(${PROJECT_NAME} 
    PUBLIC 
    $<TARGET_PROPERTY:stm32wl3x_rf_api_obj,INTERFACE_COMPILE_DEFINITIONS>
    $<TARGET_PROPERTY:components,INTERFACE_COMPILE_DEFINITIONS>
    $<TARGET_PROPERTY:cmsis,INTERFACE_COMPILE_DEFINITIONS>
    $<TARGET_PROPERTY:peripherials,INTERFACE_COMPILE_DEFINITIONS>
  )
endif()
