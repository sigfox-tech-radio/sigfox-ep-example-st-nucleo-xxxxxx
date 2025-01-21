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

set(SIGFOX_EP_LIB_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/sigfox-ep-lib)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sigfox-ep-lib EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sigfox-ep-addon-rfp EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sigfox-ep-addon-ta EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/mcu_api EXCLUDE_FROM_ALL)

if(SIGFOX_RF_API STREQUAL lr11xx_rf_api_obj)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sigfox-ep-rf-api-semtech-lr11xx EXCLUDE_FROM_ALL)
elseif(SIGFOX_RF_API STREQUAL sx126x_rf_api_obj)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sigfox-ep-rf-api-semtech-sx126x EXCLUDE_FROM_ALL)
elseif(SIGFOX_RF_API STREQUAL s2lp_rf_api_obj)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sigfox-ep-rf-api-st-s2lp EXCLUDE_FROM_ALL)
  #Patch for rf-api-st-s2lp, adding board directory
  target_include_directories( s2lp_rf_api_obj PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/sigfox-ep-rf-api-st-s2lp/inc/board
  )
endif()



