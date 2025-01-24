############################################################################
# apps/canutils/lely-canopen/Makefile
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
###########################################################################

# Standard includes

include $(APPDIR)/Make.defs

# Set up build configuration and environment

WD := ${shell echo $(CURDIR) | sed -e 's/ /\\ /g'}

LELYCANOPEN_DIR  = lely-core
LELYCANOPEN_VER  = $(patsubst "%",%,$(strip $(CONFIG_CANUTILS_LELYCANOPEN_VERSION)))
LELYCANOPEN_TAR  = $(LELYCANOPEN_VER).tar.gz
LELYCANOPEN_URL := https://gitlab.com/lely_industries/lely-core/-/archive/v$(LELYCANOPEN_TAR)

MODULE = $(CONFIG_CANUTILS_LELYCANOPEN)

# Lely CANopen tools

ifeq ($(CONFIG_CANUTILS_LELYCANOPEN_TOOLS_COCTL),y)
PROGNAME   = coctl
PRIORITY   = $(CONFIG_CANUTILS_LELYCANOPEN_TOOLS_COCTL_PRIORITY)
STACKSIZE  = $(CONFIG_CANUTILS_LELYCANOPEN_TOOLS_COCTL_STACKSIZE)
MAINSRC    = $(WD)/$(LELYCANOPEN_DIR)/tools/coctl.c
endif

$(LELYCANOPEN_TAR):
	@echo "Downloading: $(LELYCANOPEN_TAR)"
	$(Q) curl -L $(LELYCANOPEN_URL) -o $(LELYCANOPEN_TAR)

$(LELYCANOPEN_DIR): $(LELYCANOPEN_TAR)
	@echo "Unpacking: $(LELYCANOPEN_TAR) -> $(LELYCANOPEN_DIR)"
	$(Q) mkdir -p $(LELYCANOPEN_DIR)
	$(Q) tar zxf $(LELYCANOPEN_TAR) -C $(LELYCANOPEN_DIR) --strip-components=1
	$(Q) touch $(LELYCANOPEN_DIR)
	$(Q) patch -p1 -d $(LELYCANOPEN_DIR) < 0001-NuttX-port.patch

# Download and unpack tarball if no git repo found
ifeq ($(wildcard $(LELYCANOPEN_DIR)/.git),)
context:: $(LELYCANOPEN_DIR)
distclean::
	$(call DELFILE,$(LELYCANOPEN_TAR))
	$(call DELDIR,$(LELYCANOPEN_DIR))
endif

include $(APPDIR)/canutils/lely-canopen/Makefile.lely

include $(APPDIR)/Application.mk
