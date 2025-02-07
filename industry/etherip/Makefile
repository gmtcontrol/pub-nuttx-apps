############################################################################
# apps/industry/etherip/Makefile
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

ETHERIP_DIR  = opener
ETHERIP_VER := $(patsubst "%",%,$(strip $(CONFIG_ODVA_ETHERIP_VERSION)))
ETHERIP_TAR := $(ETHERIP_VER).tar.gz
ETHERIP_URL := https://github.com/EIPStackGroup/OpENer/archive/v$(ETHERIP_TAR)

$(ETHERIP_TAR):
	@echo "Downloading: $(ETHERIP_URL)"
	$(Q) curl -L $(ETHERIP_URL) -o $(ETHERIP_TAR)

$(ETHERIP_DIR): $(ETHERIP_TAR)
	@echo "Unpacking: $(ETHERIP_TAR) -> $(ETHERIP_DIR)"
	$(Q) tar zxf $(ETHERIP_TAR)
	$(Q) mv OpENer-$(ETHERIP_VER) $(ETHERIP_DIR)
	$(Q) touch $(ETHERIP_DIR)
	$(Q) patch -p1 -d $(ETHERIP_DIR) < 0001-nuttx-port.patch

# Download and unpack tarball if no git repo found
ifeq ($(wildcard $(ETHERIP_DIR)/.git),)
context:: $(ETHERIP_DIR)

distclean::
	$(call DELFILE,$(ETHERIP_TAR))
	$(call DELDIR,$(ETHERIP_DIR))
endif

include $(APPDIR)/Application.mk
