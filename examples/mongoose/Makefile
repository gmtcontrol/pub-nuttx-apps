############################################################################
# apps/examples/mongoose/Makefile
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

# Source files
CSRCS 	 =
MAINSRC  =
PROGNAME =

# Simple Web Server
ifneq ($(CONFIG_EXAMPLES_MONGOOSE_HTTPD),)
CSRCS    += httpd/net.c
MAINSRC  += httpd/httpd.c
PROGNAME += httpd_start
endif

# Websocket Server
ifneq ($(CONFIG_EXAMPLES_MONGOOSE_WEBSOCKD),)
MAINSRC  += websockd/websockd.c
PROGNAME += wsockd_start
endif

# Smart Antenna Server
ifneq ($(CONFIG_EXAMPLES_MONGOOSE_SMARTANT),)

# uORB messaging
ifneq ($(CONFIG_EXAMPLES_MONGOOSE_SMARTANT_UORB),)
MAINSRC += smart/smartd_uorb.c
endif

# FIFO messaging
ifneq ($(CONFIG_EXAMPLES_MONGOOSE_SMARTANT_FIFO),)
MAINSRC += smart/smartd_fifo.c
endif

MAINSRC  += smart/smartd_stop.c
PROGNAME += smartd_start smartd_stop
endif

# Dashboard Server
ifneq ($(CONFIG_EXAMPLES_MONGOOSE_DASHBOARD),)
CSRCS    += dash/device.c
MAINSRC  += dash/httpd.c
MAINSRC  += dash/httpd_stop.c
PROGNAME += systemd_start systemd_stop
endif

PRIORITY  = $(CONFIG_EXAMPLES_MONGOOSE_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_MONGOOSE_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_MONGOOSE)

include $(APPDIR)/netutils/mongoose/Makefile.mongoose

include $(APPDIR)/Application.mk
