############################################################################
# apps/industry/soem/Makefile
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

SOEM_DIR  = soem
SOEM_VER := $(patsubst "%",%,$(strip $(CONFIG_RTLABS_SOEM_VERSION)))
SOEM_TAR := $(SOEM_VER).tar.gz
SOEM_URL := https://github.com/OpenEtherCATsociety/SOEM/archive/refs/tags/v$(SOEM_TAR)

$(SOEM_TAR):
	@echo "Downloading: $(SOEM_URL)"
	$(Q) curl -L $(SOEM_URL) -o $(SOEM_TAR)

$(SOEM_DIR): $(SOEM_TAR)
	@echo "Unpacking: $(SOEM_TAR) -> $(SOEM_DIR)"
	$(Q) tar zxf $(SOEM_TAR)
	$(Q) mv SOEM-$(SOEM_VER) $(SOEM_DIR)
	$(Q) touch $(SOEM_DIR)
	$(Q) patch -p1 -d $(SOEM_DIR) < 0001-nuttx-port.patch

# Download and unpack tarball if no git repo found
ifeq ($(wildcard $(SOEM_DIR)/.git),)
context:: $(SOEM_DIR)

distclean::
	$(call DELFILE,$(SOEM_TAR))
	$(call DELDIR,$(SOEM_DIR))
endif

include $(APPDIR)/Application.mk
