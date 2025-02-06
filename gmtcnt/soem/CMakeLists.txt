# ##############################################################################
# apps/gmtcnt/soem/CMakeLists.txt
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

if(CONFIG_GMTCNT_SOEM)

if(CONFIG_GMTCNT_SOEM_SERVO)
	nuttx_add_application(
		MODULE
		${CONFIG_GMTCNT_SOEM}
		NAME
		soem_servo
		PRIORITY
		${CONFIG_GMTCNT_SOEM_PRIORITY}
		STACKSIZE
		${CONFIG_GMTCNT_SOEM_STACKSIZE}
		SRCS
		servo/servo.c)
	endif

	if(CONFIG_GMTCNT_SOEM_SIMPLE)
		nuttx_add_application(
			MODULE
			${CONFIG_GMTCNT_SOEM}
			NAME
			soem_simple
			PRIORITY
			${CONFIG_GMTCNT_SOEM_PRIORITY}
			STACKSIZE
			${CONFIG_GMTCNT_SOEM_STACKSIZE}
			SRCS
			simple_test/simple_test.c)
	endif

	if(CONFIG_GMTCNT_SOEM_RED)
		nuttx_add_application(
			MODULE
			${CONFIG_GMTCNT_SOEM}
			NAME
			soem_red
			PRIORITY
			${CONFIG_GMTCNT_SOEM_PRIORITY}
			STACKSIZE
			${CONFIG_GMTCNT_SOEM_STACKSIZE}
			SRCS
			red_test/red_test.c)
	endif

	if(CONFIG_GMTCNT_SOEM_SLINFO)
		nuttx_add_application(
			MODULE
			${CONFIG_GMTCNT_SOEM}
			NAME
			soem_slinfo
			PRIORITY
			${CONFIG_GMTCNT_SOEM_PRIORITY}
			STACKSIZE
			${CONFIG_GMTCNT_SOEM_STACKSIZE}
			SRCS
			slaveinfo/slaveinfo.c)
	endif

	if(CONFIG_GMTCNT_SOEM_EBOX)
		nuttx_add_application(
			MODULE
			${CONFIG_GMTCNT_SOEM}
			NAME
			soem_ebox
			PRIORITY
			${CONFIG_GMTCNT_SOEM_PRIORITY}
			STACKSIZE
			${CONFIG_GMTCNT_SOEM_STACKSIZE}
			SRCS
			ebox/ebox.c)
	endif

	if(CONFIG_GMTCNT_SOEM_EOE)
		nuttx_add_application(
			MODULE
			${CONFIG_GMTCNT_SOEM}
			NAME
			soem_eoe
			PRIORITY
			${CONFIG_GMTCNT_SOEM_PRIORITY}
			STACKSIZE
			${CONFIG_GMTCNT_SOEM_STACKSIZE}
			SRCS
			eoe_test/eoe_test.c)
	endif

	if(CONFIG_GMTCNT_SOEM_E2PROMTOOL)
		nuttx_add_application(
			MODULE
			${CONFIG_GMTCNT_SOEM}
			NAME
			soem_e2promtool
			PRIORITY
			${CONFIG_GMTCNT_SOEM_PRIORITY}
			STACKSIZE
			${CONFIG_GMTCNT_SOEM_STACKSIZE}
			SRCS
			eepromtool/eepromtool.c)
	endif

	if(CONFIG_GMTCNT_SOEM_ALIASTOOL)
		nuttx_add_application(
			MODULE
			${CONFIG_GMTCNT_SOEM}
			NAME
			soem_aliastool
			PRIORITY
			${CONFIG_GMTCNT_SOEM_PRIORITY}
			STACKSIZE
			${CONFIG_GMTCNT_SOEM_STACKSIZE}
			SRCS
			aliastool/aliastool.c)
	endif

	if(CONFIG_GMTCNT_SOEM_FWUPDATE)
		nuttx_add_application(
			MODULE
			${CONFIG_GMTCNT_SOEM}
			NAME
			soem_fwupdate
			PRIORITY
			${CONFIG_GMTCNT_SOEM_PRIORITY}
			STACKSIZE
			${CONFIG_GMTCNT_SOEM_STACKSIZE}
			SRCS
			firm_update/firm_update.c)
	endif

	endif()
