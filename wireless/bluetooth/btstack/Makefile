############################################################################
# apps/wireless/bluetooth/btstack/Makefile
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

BTSTACK_DIR  = btstack
BTSTACK_VER := $(patsubst "%",%,$(strip $(CONFIG_BTSTACK_VERSION)))
BTSTACK_TAR := $(BTSTACK_VER).tar.gz
BTSTACK_URL := https://github.com/bluekitchen/btstack/archive/v$(BTSTACK_TAR)

# GATT profile data files
BTSTACK_ROOT = $(APPDIR)/wireless/bluetooth/btstack/btstack
PROFILE_GEN  = python3 $(BTSTACK_ROOT)/tool/compile_gatt.py $(1) $(1:.gatt=.h)
PROFILE_DIR  = $(APPDIR)/ardusimple/btstack/data
PROFILE_SRC  = $(wildcard $(PROFILE_DIR)/*.gatt)

$(BTSTACK_TAR):
	@echo "Downloading: $(BTSTACK_URL)"
	$(Q) curl -L $(BTSTACK_URL) -o $(BTSTACK_TAR)

$(BTSTACK_DIR): $(BTSTACK_TAR)
	@echo "Unpacking: $(BTSTACK_TAR) -> $(BTSTACK_DIR)"
	$(Q) tar zxf $(BTSTACK_TAR)
	$(Q) mv btstack-$(BTSTACK_VER) $(BTSTACK_DIR)
	$(Q) touch $(BTSTACK_DIR)
	$(Q) patch -p1 -d $(BTSTACK_DIR) < 0001-nuttx-port.patch

# Download and unpack tarball if no git repo found
ifeq ($(wildcard $(BTSTACK_DIR)/.git),)
context:: $(BTSTACK_DIR)
	$(Q) $(foreach SRC, $(PROFILE_SRC), $(call PROFILE_GEN, $(SRC));)

distclean::
	$(call DELFILE,$(BTSTACK_TAR))
	$(call DELDIR,$(BTSTACK_DIR))
	$(foreach OBJ, $(wildcard $(PROFILE_DIR)/*.h), $(call DELFILE, $(OBJ)))
else

context::
	$(Q) $(foreach SRC, $(PROFILE_SRC), $(call PROFILE_GEN, $(SRC));)

endif

include $(APPDIR)/Application.mk
