# ##############################################################################
# apps/wireless/bluetooth/btstack/CMakeLists.txt
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

if(CONFIG_BTSTACK)

  # ############################################################################
  # Config and Fetch btstack
  # ############################################################################

  set(BTSTACK_DIR
    ${CMAKE_CURRENT_LIST_DIR}/btstack
  )

  if(NOT EXISTS ${BTSTACK_DIR})

    set(BTSTACK_VER
      ${CONFIG_BTSTACK_VERSION}
    )

    set(BTSTACK_URL
      https://github.com/bluekitchen/btstack/archive/${BTSTACK_VER}.tar.gz
    )

    FetchContent_Declare(
      btstack_fetch
      URL ${BTSTACK_URL} SOURCE_DIR ${BTSTACK_DIR}
          BINARY_DIR ${CMAKE_BINARY_DIR}/apps/wireless/btstack/btstack
      PATCH_COMMAND
        patch -p0 -d ${BTSTACK_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0001-nuttx-port.patch
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(btstack_fetch)

    if(NOT btstack_fetch_POPULATED)
      FetchContent_Populate(btstack_fetch)
    endif()
  endif()

  nuttx_add_library(btstack)

  # ############################################################################
  # Include Directory
  # ############################################################################

  set(INCDIR ${BTSTACK_DIR})

  nuttx_export_header(TARGET btstack INCLUDE_DIRECTORIES ${INCDIR})

  # ############################################################################
  # Sources
  # ############################################################################

  # ############################################################################
  # Applications Configuration
  # ############################################################################

  set(CFLAGS -Wno-pointer-to-int-cast -Wno-undef -Wno-format -Wno-unused-but-set-variable)

  # ############################################################################
  # Library Configuration
  # ############################################################################

  target_sources(btstack PRIVATE ${CSRCS})
  target_include_directories(btstack PRIVATE ${INCDIR})
  if(CFLAGS)
    target_compile_options(btstack PRIVATE ${CFLAGS})
  endif()

endif()
