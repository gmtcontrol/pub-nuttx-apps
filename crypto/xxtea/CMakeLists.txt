# ##############################################################################
# apps/crypto/xxtea/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

if(CONFIG_XXTEA)

  # ############################################################################
  # Config and Fetch xxtea
  # ############################################################################
  set(XXTEA_DIR ${CMAKE_CURRENT_LIST_DIR}/xxtea)

  if(NOT EXISTS ${XXTEA_DIR})
    set(XXTEA_VERSION ${CONFIG_XXTEA_VERSION})

    set(XXTEA_URL
      https://github.com/xxtea/xxtea-c/archive/refs/tags/v${XXTEA_VERSION}.zip
    )
    FetchContent_Declare(
      tinycrypt_fetch
      URL ${XXTEA_URL} SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/xxtea
          BINARY_DIR ${CMAKE_BINARY_DIR}/apps/crypto/xxtea/xxtea
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(tinycrypt_fetch)

    if(NOT tinycrypt_fetch_POPULATED)
      FetchContent_Populate(tinycrypt_fetch)
    endif()
  endif()

  nuttx_add_library(xxtea STATIC)

  # ############################################################################
  # Include Directory
  # ############################################################################

  set(INCDIR ${XXTEA_DIR})

  nuttx_export_header(TARGET xxtea INCLUDE_DIRECTORIES ${INCDIR})

  # ############################################################################
  # Sources
  # ############################################################################

  set(CSRCS ${XXTEA_DIR}/xxtea.c)

  # ############################################################################
  # Library Configuration
  # ############################################################################

  target_sources(xxtea PRIVATE ${CSRCS})
  target_include_directories(xxtea PRIVATE ${INCDIR})
  if(CFLAGS)
    target_compile_options(xxtea PRIVATE ${CFLAGS})
  endif()
endif()
