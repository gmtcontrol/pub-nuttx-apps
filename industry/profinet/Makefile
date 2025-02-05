############################################################################
# apps/industry/profinet/Makefile
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

PROFINET_DIR  = p-net
PROFINET_VER := $(patsubst "%",%,$(strip $(CONFIG_RTLABS_PROFINET_VERSION)))
PROFINET_TAR := $(PROFINET_VER).tar.gz
PROFINET_URL := https://github.com/rtlabs-com/p-net/archive/refs/tags/public/v$(PROFINET_TAR)

$(PROFINET_TAR):
	@echo "Downloading: $(PROFINET_URL)"
	$(Q) curl -L $(PROFINET_URL) -o $(PROFINET_TAR)

$(PROFINET_DIR): $(PROFINET_TAR)
	@echo "Unpacking: $(PROFINET_TAR) -> $(PROFINET_DIR)"
	$(Q) tar zxf $(PROFINET_TAR)
	$(Q) mv p-net-public-v$(PROFINET_VER) $(PROFINET_DIR)
	$(Q) touch $(PROFINET_DIR)
	$(Q) patch -p1 -d $(PROFINET_DIR) < 0001-nuttx-port.patch

# Download and unpack tarball if no git repo found
ifeq ($(wildcard $(PROFINET_DIR)/.git),)
context:: $(PROFINET_DIR)

distclean::
	$(call DELFILE,$(PROFINET_TAR))
	$(call DELDIR,$(PROFINET_DIR))
endif

include $(APPDIR)/Application.mk
