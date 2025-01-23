# ##############################################################################
# apps/examples/mongoose/CMakeLists.txt
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

if (CONFIG_EXAMPLES_MONGOOSE)

	# Simple Web server
  if (CONFIG_EXAMPLES_MONGOOSE_HTTPD)
		nuttx_add_application(
			NAME
			httpd_start
      STACKSIZE
      ${CONFIG_EXAMPLES_MONGOOSE_STACKSIZE}
      PRIORITY
      ${CONFIG_EXAMPLES_MONGOOSE_PRIORITY}
      SRCS
			httpd/net.c httpd/httpd.c)
	endif

	# Websocket server
	if (CONFIG_EXAMPLES_MONGOOSE_WEBSOCKD)
		nuttx_add_application(
			NAME
			wsockd_start
      STACKSIZE
      ${CONFIG_EXAMPLES_MONGOOSE_STACKSIZE}
      PRIORITY
      ${CONFIG_EXAMPLES_MONGOOSE_PRIORITY}
			SRCS
			websockd/websockd.c)
	endif

  # Smart Antenna server
	if (CONFIG_EXAMPLES_MONGOOSE_SMARTANT)

    if (CONFIG_EXAMPLES_MONGOOSE_SMARTANT_UORB)
      nuttx_add_application(
        NAME
        smartd_start
        STACKSIZE
        ${CONFIG_EXAMPLES_MONGOOSE_STACKSIZE}
        PRIORITY
        ${CONFIG_EXAMPLES_MONGOOSE_PRIORITY}
        SRCS
        smart/smartd_uorb.c)
      endif

      nuttx_add_application(
        NAME
        smartd_stop
        STACKSIZE
        ${DEFAULT_TASK_STACKSIZE}
        PRIORITY
        ${SCHED_PRIORITY_DEFAULT}
        SRCS
        smart/smartd_stop.c)
      endif

    if (CONFIG_EXAMPLES_MONGOOSE_SMARTANT_FIFO)
      nuttx_add_application(
        NAME
        smartd_start
        STACKSIZE
        ${CONFIG_EXAMPLES_MONGOOSE_STACKSIZE}
        PRIORITY
        ${CONFIG_EXAMPLES_MONGOOSE_PRIORITY}
        SRCS
        smart/smartd_fifo.c)

      nuttx_add_application(
        NAME
        smartd_stop
        STACKSIZE
        ${DEFAULT_TASK_STACKSIZE}
        PRIORITY
        ${SCHED_PRIORITY_DEFAULT}
        SRCS
        smart/smartd_stop.c)
    endif
  endif

	# Dashboard server
	if (CONFIG_EXAMPLES_MONGOOSE_DASHBOARD)
		nuttx_add_application(
			NAME
			systemd_start
      STACKSIZE
      ${CONFIG_EXAMPLES_MONGOOSE_STACKSIZE}
      PRIORITY
      ${CONFIG_EXAMPLES_MONGOOSE_PRIORITY}
			SRCS
			dash/httpd.c dash/device.c)

    nuttx_add_application(
      NAME
      systemd_stop
      STACKSIZE
      ${DEFAULT_TASK_STACKSIZE}
      PRIORITY
      ${SCHED_PRIORITY_DEFAULT}
      SRCS
      dash/httpd_stop.c)
	endif

endif()
