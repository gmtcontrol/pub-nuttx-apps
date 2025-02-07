/****************************************************************************
 * apps/boot/tinyboot/include/tinyboot.h
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

#ifndef __BOOT_TINYBOOT_INCLUDE_TINYBOOT_H
#define __BOOT_TINYBOOT_INCLUDE_TINYBOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <assert.h>
#include <stdbool.h>
#include <fcntl.h>

#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TINYBOOT_PRIMARY_SLOT_NUM   (0)
#define TINYBOOT_SECONDARY_SLOT_NUM (1)
#define TINYBOOT_TERTIARY_SLOT_NUM  (2)

/* Offsets to write pages containing confirmed and updated flags. These
 * pages are located at the end of the partition, therefore index 0 means
 * the first page from the end.
 */

#define TINYBOOT_CONFIRMED_PAGE_INDEX (0)
#define TINYBOOT_UPDATED_PAGE_INDEX   (1)

#define TINYBOOT_HEADER_MAGIC     0x534f584e /* NXOS. */
#define TINYBOOT_HEADER_MAGIC_INV 0xaca0abb1 /* NXOS inverted. This is used
                                            * for images uploaded directly
                                            * to the primary flash with
                                            * the debugger. These images
                                            * does not have precalculated
                                            * CRC and flags at the
                                            * end of the partition, but
                                            * are considered to be valid.
                                            */

#define TINYBOOT_HEADER_PRERELEASE_MAXLEN 110

#define TINYBOOT_LEDS_NUM   (3)
#define TINYBOOT_LEDS_ON    (0)
#define TINYBOOT_LEDS_OFF   (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum tinyboot_update_type
{
  TINYBOOT_UPDATE_TYPE_NONE = 0,    /* No action to do */
  TINYBOOT_UPDATE_TYPE_UPDATE = 1,  /* Update will take place upon reboot */
  TINYBOOT_UPDATE_TYPE_REVERT = 2,  /* Revert will take place upon reboot */
};

/* Versioning is according to Semantic Versioning 2.0.0
 * refer to (https://semver.org/spec/v2.0.0.html)
 */

struct tinyboot_img_version
{
  uint16_t major;        /* MAJOR version */
  uint16_t minor;        /* MINOR version */
  uint16_t patch;        /* PATCH version */

  char pre_release[TINYBOOT_HEADER_PRERELEASE_MAXLEN];  /* Additional pre-release version */
};

struct tinyboot_img_header
{
  uint32_t magic;  /* Header magic */
  uint32_t size;   /* Image size (excluding the header) */
  uint32_t crc;    /* CRC32 of image (excluding the header). */

  struct tinyboot_img_version img_version; /* Image version */
};
static_assert(CONFIG_TINYBOOT_HEADER_SIZE > sizeof(struct tinyboot_img_header),
              "CONFIG_TINYBOOT_HEADER_SIZE has to be larger than"
              "sizeof(struct tinyboot_img_header)");

struct tinyboot_state
{
  int fds[TINYBOOT_LEDS_NUM];           /* LED file descriptor */
  int update;                           /* Number of update slot */
  int recovery;                         /* Number of recovery slot */
  bool recovery_valid;                  /* True if recovery image contains valid recovery */
  bool primary_confirmed;               /* True if primary slot is confirmed */
  enum tinyboot_update_type next_boot;  /* True if update slot has a valid image */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tinyboot_get_state
 *
 * Description:
 *   Gets the current bootloader state and stores it in the tinyboot_state
 *   structure passed as an argument. This function may be used to determine
 *   which slot is update slot and where should application save incoming
 *   firmware.
 *
 * Input parameters:
 *   state: The pointer to tinyboot_state structure. The state is stored here.
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int tinyboot_get_state(struct tinyboot_state *state);

/****************************************************************************
 * Name: tinyboot_get_confirm
 *
 * Description:
 *   This function can be used to determine whether primary image is
 *   confirmed or not. This provides more direct access to confirm
 *   state compared to tinyboot_get_state function that returns the full
 *   state of the bootloader.
 *
 * Returned Value:
 *   1 means confirmed, 0 not confirmed, -1 and sets errno on failure.
 *
 ****************************************************************************/

int tinyboot_get_confirm(void);

/****************************************************************************
 * Name: tinyboot_confirm
 *
 * Description:
 *   Confirms the image currently located in primary partition and marks
 *   its copy in update partition as a recovery.
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int tinyboot_confirm(struct tinyboot_state *state);

/****************************************************************************
 * Name: tinyboot_perform_swap
 *
 * Description:
 *   Checks for the possible firmware update and performs it by copying
 *   update image to primary slot or recovery image to primary slot in case
 *   of the revert. In any situation, this function ends with the valid
 *   image in primary slot.
 *
 *   This is an entry point function that should be called from the
 *   bootloader application.
 *
 * Input parameters:
 *   check_only: Only repairs corrupted update, but do not start another one
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int tinyboot_perform_update(bool check_only,
                            struct tinyboot_state *state);

#endif /* __BOOT_TINYBOOT_INCLUDE_TINYBOOT_H */
