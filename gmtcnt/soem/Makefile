############################################################################
# apps/gmtcnt/soem/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(APPDIR)/Make.defs

# EtherCAT™ applications

MODULE 		= $(CONFIG_GMTCNT_SOEM)
PRIORITY  = $(CONFIG_GMTCNT_SOEM_PRIORITY)
STACKSIZE = $(CONFIG_GMTCNT_SOEM_STACKSIZE)
PROGNAME  =
MAINSRC   = 

ifneq ($(CONFIG_GMTCNT_SOEM_SERVO),)
MAINSRC  += servo/servo.c
PROGNAME += soem_servo
endif

ifneq ($(CONFIG_GMTCNT_SOEM_SIMPLE),)
MAINSRC  += simple_test/simple_test.c
PROGNAME += soem_simple
endif

ifneq ($(CONFIG_GMTCNT_SOEM_RED),)
MAINSRC  += red_test/red_test.c
PROGNAME += soem_red
endif

ifneq ($(CONFIG_GMTCNT_SOEM_SLINFO),)
MAINSRC  += slaveinfo/slaveinfo.c
PROGNAME += soem_slinfo
endif

ifneq ($(CONFIG_GMTCNT_SOEM_EBOX),)
MAINSRC  += ebox/ebox.c
PROGNAME += soem_ebox
endif

ifneq ($(CONFIG_GMTCNT_SOEM_EOE),)
MAINSRC  += eoe_test/eoe_test.c
PROGNAME += soem_eoe
endif

ifneq ($(CONFIG_GMTCNT_SOEM_E2PROMTOOL),)
MAINSRC  += eepromtool/eepromtool.c
PROGNAME += soem_e2promtool
endif

ifneq ($(CONFIG_GMTCNT_SOEM_ALIASTOOL),)
MAINSRC  += aliastool/aliastool.c
PROGNAME += soem_aliastool
endif

ifneq ($(CONFIG_GMTCNT_SOEM_FWUPDATE),)
MAINSRC  += firm_update/firm_update.c
PROGNAME += soem_fwupdate
endif

###########################################################
# Application specific definitions
###########################################################

include $(APPDIR)/industry/soem/Makefile.soem

include $(APPDIR)/Application.mk
