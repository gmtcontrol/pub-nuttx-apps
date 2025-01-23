############################################################################
# apps/crypto/xxtea/Makefile
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

XXTEA_VER  = $(patsubst "%",%,$(strip $(CONFIG_XXTEA_VERSION)))
XXTEA_ZIP  = v$(XXTEA_VER).zip
XXTEA_URL ?= "https://github.com/xxtea/xxtea-c/archive/refs/tags/$(XXTEA_ZIP)"

XXTEA_UNPACKNAME = xxtea
UNPACK ?= unzip -q -o

$(XXTEA_ZIP):
	@echo "Downloading: $(XXTEA_URL)/$(XXTEA_ZIP)"
	$(Q) curl -O -L $(XXTEA_URL)

$(XXTEA_UNPACKNAME): $(XXTEA_ZIP)
	@echo "Unpacking: $(XXTEA_ZIP) -> $(XXTEA_UNPACKNAME)"
	$(Q) $(UNPACK) $(XXTEA_ZIP)
	$(Q) mv	xxtea-c-$(XXTEA_VER) $(XXTEA_UNPACKNAME)
	$(Q) touch $(XXTEA_UNPACKNAME)

ifeq ($(wildcard $(XXTEA_UNPACKNAME)/.git),)
context:: $(XXTEA_UNPACKNAME)

distclean::
	$(Q) rm -rf $(XXTEA_UNPACKNAME)
endif

CSRCS  += xxtea/xxtea.c
CFLAGS += -DLITTLE_ENDIAN=1234 -DBIG_ENDIAN=4321 -DPDP_ENDIAN=3412
CFLAGS += -DBYTE_ORDER=LITTLE_ENDIAN

include $(APPDIR)/Application.mk
