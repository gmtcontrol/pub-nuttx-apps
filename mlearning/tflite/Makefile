############################################################################
# apps/mlearning/tflite/Makefile
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

SRC = tflite/tensorflow/lite

#CSRCS   +=$(SRC)/gemm.c
#CSRCS   +=$(SRC)/utils.c

CFLAGS += -Wno-shadow -Wno-strict-prototypes -Wno-unknown-pragmas

MODULE = $(CONFIG_TFLITE_MICRO)

tflite.zip:
	$(Q) curl -L https://github.com/tensorflow/tflite-micro/archive/refs/heads/$(CONFIG_TFLITE_MICRO_VER).zip -o tflite.zip
	$(Q) unzip -o tflite.zip
	$(Q) mv tflite-micro-$(CONFIG_TFLITE_MICRO_VER) tflite

# Download and unpack tarball if no git repo found
ifeq ($(wildcard tflite/.git),)
context:: tflite.zip

distclean::
	$(call DELDIR, tflite)
	$(call DELFILE, tflite.zip)
endif

include $(APPDIR)/Application.mk
