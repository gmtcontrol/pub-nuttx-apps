/****************************************************************************
 * apps/gmtcnt/plclogic/include/plclogic.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __GMTCNT_PLCLOGIC_INCLUDE_PLCLOGIC_H
#define __GMTCNT_PLCLOGIC_INCLUDE_PLCLOGIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <assert.h>
#include <stdbool.h>
#include <fcntl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PLCLOGIC_FIRMWARE_SLOT_NUM   	(0)
#define PLCLOGIC_APPLICATION_SLOT_NUM (1)

/* Offsets to write pages containing confirmed and updated flags. These
 * pages are located at the end of the partition, therefore index 0 means
 * the first page from the end.
 */

#define PLCLOGIC_CONFIRMED_PAGE_INDEX (0)
#define PLCLOGIC_UPDATED_PAGE_INDEX   (1)

#define PLCLOGIC_HEADER_MAGIC     0x534f584e /* NXOS. */
#define PLCLOGIC_HEADER_MAGIC_INV 0xaca0abb1 /* NXOS inverted. This is used
                                            * for images uploaded directly
                                            * to the primary flash with
                                            * the debugger. These images
                                            * does not have precalculated
                                            * CRC and flags at the
                                            * end of the partition, but
                                            * are considered to be valid.
                                            */

#define PLCLOGIC_HEADER_PRERELEASE_MAXLEN 110

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Versioning is according to Semantic Versioning 2.0.0
 * refer to (https://semver.org/spec/v2.0.0.html)
 */

struct plclogic_img_version
{
  uint16_t major;        /* MAJOR version */
  uint16_t minor;        /* MINOR version */
  uint16_t patch;        /* PATCH version */

  char pre_release[PLCLOGIC_HEADER_PRERELEASE_MAXLEN];  /* Additional pre-release version */
};

struct plclogic_img_header
{
  uint32_t magic;  /* Header magic */
  uint32_t size;   /* Image size (excluding the header) */
  uint32_t crc;    /* CRC32 of image (excluding the header). */

  struct plclogic_img_version img_version; /* Image version */
};
static_assert(CONFIG_GMTCNT_PLCLOGIC_HEADER_SIZE > sizeof(struct plclogic_img_header),
              "CONFIG_GMTCNT_PLCLOGIC_HEADER_SIZE has to be larger than"
              "sizeof(struct plclogic_img_header)");

struct plclogic_state
{
	uint32_t temp;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: plclogic_get_state
 *
 * Description:
 *   Gets the current application state and stores it in the plclogic_state
 *   structure passed as an argument. This function may be used to determine
 *   which slot is update slot and where should application save incoming
 *   firmware.
 *
 * Input parameters:
 *   state: The pointer to plclogic_state structure. The state is stored here.
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int plclogic_get_state(struct plclogic_state *state);

/****************************************************************************
 * Name: plclogic_get_confirm
 *
 * Description:
 *   This function can be used to determine whether applicaiton image is
 *   confirmed or not. This provides more direct access to confirm
 *   state compared to plclogic_get_state function that returns the full
 *   state of the bootloader.
 *
 * Returned Value:
 *   1 means confirmed, 0 not confirmed, -1 and sets errno on failure.
 *
 ****************************************************************************/

int plclogic_get_confirm(void);

/****************************************************************************
 * Name: plclogic_confirm
 *
 * Description:
 *   Confirms the image currently located in application partition and marks
 *   its copy in update partition as a recovery.
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int plclogic_confirm(struct plclogic_state *state);

#endif /* __GMTCNT_PLCPLCLOGIC_INCLUDE_PLCPLCLOGIC_H */
