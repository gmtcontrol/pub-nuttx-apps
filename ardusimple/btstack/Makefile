############################################################################
# apps/ardusimple/btstack/Makefile
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

# BTStack built-in application info

PROGNAME  = $(CONFIG_ARDUSIMPLE_BTSTACK_PROGNAME)
PRIORITY  = $(CONFIG_ARDUSIMPLE_BTSTACK_PRIORITY)
STACKSIZE = $(CONFIG_ARDUSIMPLE_BTSTACK_STACKSIZE)
MODULE    = $(CONFIG_ARDUSIMPLE_BTSTACK)
CSRCS 		=

###########################################################
# BTStack GATT applications files
###########################################################

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GATT),y)

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GATT_COUNTER),y)
CSRCS += gatt/gatt_counter.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GATT_BROWSER),y)
CSRCS += gatt/gatt_browser.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GATT_BATT_QUERY),y)
CSRCS += gatt/gatt_battery_query.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GATT_HEART_RATE_CLIENT),y)
CSRCS += gatt/gatt_heart_rate_client.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GATT_HEART_RATE_SERVER),y)
CSRCS += gatt/gatt_heart_rate_server.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GATT_DEVINFO_QUERY),y)
CSRCS += gatt/gatt_device_information_query.c
endif

endif # CONFIG_ARDUSIMPLE_BTSTACK_GATT

###########################################################
# BTStack SPP applications files
###########################################################

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP),y)

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_MAPKIT),y)
CSRCS += spp/spp_mapkit.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_NORDIC_LE_MAPKIT),y)
CSRCS += spp/nordic_spp_le_mapkit.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_UBLOX_LE_MAPKIT),y)
CSRCS += spp/ublox_spp_le_mapkit.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_COUNTER),y)
CSRCS += spp/spp_counter.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_STREAMER),y)
CSRCS += spp/spp_streamer.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_STREAMER_CLIENT),y)
CSRCS += spp/spp_streamer_client.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_FLOWCTRL),y)
CSRCS += spp/spp_flowcontrol.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_GATT_COUNTER),y)
CSRCS += spp/spp_and_gatt_counter.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_GATT_STREAMER),y)
CSRCS += spp/spp_and_gatt_streamer.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_UBLOX_LE_COUNTER),y)
CSRCS += spp/ublox_spp_le_counter.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_NORDIC_LE_COUNTER),y)
CSRCS += spp/nordic_spp_le_counter.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_SPP_NORDIC_LE_STREAMER),y)
CSRCS += spp/nordic_spp_le_streamer.c
endif

endif # CONFIG_ARDUSIMPLE_BTSTACK_SPP

###########################################################
# BTStack HID applications files
###########################################################

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_HID),y)

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_HID_HOST),y)
CSRCS += hid/hid_host_demo.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_HID_MOUSE),y)
CSRCS += hid/hid_mouse_demo.c
endif

ifeq ($(EXAMPLES_BTSTACK_HID_KEYBOARD),y)
CSRCS += hid/hid_keyboard_demo.c
endif

endif # CONFIG_ARDUSIMPLE_BTSTACK_HID

###########################################################
# BTStack GAP applications files
###########################################################

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GAP),y)

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GAP_INQUIRY),y)
CSRCS += gap/gap_inquiry.c
endif

ifeq ($(CONFIG_ARDUSIMPLE_BTSTACK_GAP_ADVER),y)
CSRCS += gap/gap_le_advertisements.c
endif

endif # CONFIG_ARDUSIMPLE_BTSTACK_GAP

###########################################################
# BTStack main file
###########################################################

MAINSRC = btstack_main.c

###########################################################
# BTStack specific definitions
###########################################################

include $(APPDIR)/wireless/bluetooth/btstack/Makefile.btstack

include $(APPDIR)/Application.mk
