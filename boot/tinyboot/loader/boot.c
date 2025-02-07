/****************************************************************************
 * apps/boot/tinyboot/loader/boot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <errno.h>
#include <syslog.h>
#include <sys/param.h>

#include <nuttx/crc32.h>
#include <tinyboot.h>

#ifdef CONFIG_LIBFLASH
#include <flash/partition.h>
#else
#include "flash.h"
#endif /* CONFIG_LIBFLASH */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_TINYBOOT_PREVENT_DOWNGRADE
  # warning "Downgrade prevention currently ignores prerelease."
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool get_image_flag(int fd, int index)
{
  uint8_t flag;
  struct flash_partition_info info;

  if (flash_partition_info(fd, &info) < 0)
    {
      return false;
    }

  if (flash_partition_read(fd, &flag, 1,
                           info.size - info.blocksize * (index + 1)) < 0)
    {
      return false;
    }

  return flag == 0xfe;
}

static inline int set_image_flag(int fd, int index)
{
  uint8_t flag;
  struct flash_partition_info info;

  if (flash_partition_info(fd, &info) < 0)
    {
      return ERROR;
    }

  flag = 0xfe;
  return flash_partition_write(fd, &flag, 1,
                               info.size - (info.blocksize * (index + 1)));
}

static inline void get_image_header(int fd, struct tinyboot_img_header *header)
{
  int ret;
  ret = flash_partition_read(fd, header, sizeof *header, 0);
  if (ret < 0)
    {
      /* Something went wrong, treat the partition as empty. */

      memset(header, 0, sizeof *header);
    }
}

static inline bool validate_image_header(struct tinyboot_img_header *header)
{
  return header->magic == TINYBOOT_HEADER_MAGIC ||
         header->magic == TINYBOOT_HEADER_MAGIC_INV;
}

static uint32_t calculate_crc(int fd, struct tinyboot_img_header *header)
{
  char *buf;
  int remain;
  int readsiz;
  off_t off;
  uint32_t crc;
  struct flash_partition_info info;

  if (flash_partition_info(fd, &info) < 0)
    {
      return false;
    }

  buf = malloc(info.blocksize);
  if (!buf)
    {
      return false;
    }

  crc = 0xffffffff;
  remain = header->size;
  off = CONFIG_TINYBOOT_HEADER_SIZE;
  while (remain > 0)
    {
      readsiz = remain > info.blocksize ? info.blocksize : remain;
      if (flash_partition_read(fd, buf, readsiz, off) != 0)
        {
          free(buf);
          return 0xffffffff;
        }

      off += readsiz;
      remain -= readsiz;
      crc = crc32part((uint8_t *)buf, readsiz, crc);
    }

  free(buf);
  return ~crc;
}

static int copy_partition(int  from,
                          int  where,
                          bool bulkerase,
                          struct tinyboot_state *state)
{
  struct tinyboot_img_header header;
  struct flash_partition_info info_from;
  struct flash_partition_info info_where;
  uint32_t crc;
  uint32_t magic;
  int readsiz;
  int remain;
  int blocksize;
  off_t off;
  char *buf;

  get_image_header(from, &header);

  if (flash_partition_info(from, &info_from) < 0)
    {
      return ERROR;
    }

  if (flash_partition_info(where, &info_where) < 0)
    {
      return ERROR;
    }

  if ((header.size + CONFIG_TINYBOOT_HEADER_SIZE) > info_where.size)
    {
      return ERROR;
    }

  blocksize = MAX(info_from.blocksize, 4096);

  buf = malloc(blocksize);
  if (!buf)
    {
      return ERROR;
    }

  /* Indicate the erase is in progress */

  ioctl(state->fds[0], GPIOC_WRITE, TINYBOOT_LEDS_ON);

  if (bulkerase)
    {
      if (flash_partition_erase(where) < 0)
        {
          return ERROR;
        }
    }
  else
    {
      if (flash_partition_erase_last_sector(where) < 0)
        {
          return ERROR;
        }
    }

  /* Indicate the erase is done */

  ioctl(state->fds[0], GPIOC_WRITE, TINYBOOT_LEDS_OFF);

  remain = header.size + CONFIG_TINYBOOT_HEADER_SIZE;
  off = 0;
  if (header.magic == TINYBOOT_HEADER_MAGIC_INV)
    {
      /* This means we are doing a recovery of a primary image
       * without the precalculated CRC. Calculate CRC and insert it
       * into the recovery image header. Also flip header's magic to
       * indicate this is an image with valid CRC.
       */

      magic = TINYBOOT_HEADER_MAGIC;
      crc = calculate_crc(from, &header);
      if (flash_partition_read(from, buf, blocksize, 0) < 0)
        {
          free(buf);
          return ERROR;
        }

      memcpy(buf + offsetof(struct tinyboot_img_header, magic), &magic,
             sizeof magic);
      memcpy(buf + offsetof(struct tinyboot_img_header, crc), &crc,
             sizeof crc);
      if (flash_partition_write(where, buf, blocksize, 0) < 0)
        {
          free(buf);
          return ERROR;
        }

      off += blocksize;
      remain -= blocksize;
    }

  bool led = TINYBOOT_LEDS_ON;

  while (remain > 0)
    {
      readsiz = remain > blocksize ? blocksize : remain;
      if (flash_partition_read(from, buf, readsiz, off) < 0)
        {
          free(buf);
          return ERROR;
        }

      if (flash_partition_write(where, buf, readsiz, off) < 0)
        {
          free(buf);
          return ERROR;
        }

      off += readsiz;
      remain -= readsiz;

      /* Indicate the writing is in progress */

      ioctl(state->fds[1], GPIOC_WRITE, led);
      led ^= 1;
    }

  /* Indicate the writing is done */

  ioctl(state->fds[1], GPIOC_WRITE, TINYBOOT_LEDS_OFF);

  if (header.magic != TINYBOOT_HEADER_MAGIC_INV)
    {
      /* Copy currently set flags but only if the image has
       * precalculated CRC.
       */

      if (get_image_flag(from, TINYBOOT_UPDATED_PAGE_INDEX))
        {
          set_image_flag(where, TINYBOOT_UPDATED_PAGE_INDEX);
        }

      if (get_image_flag(from, TINYBOOT_CONFIRMED_PAGE_INDEX))
        {
          set_image_flag(where, TINYBOOT_CONFIRMED_PAGE_INDEX);
        }
    }
  else
    {
      set_image_flag(where, TINYBOOT_CONFIRMED_PAGE_INDEX);
    }

  free(buf);
  return OK;
}

static bool validate_image(int fd)
{
  struct tinyboot_img_header header;

  get_image_header(fd, &header);
  if (!validate_image_header(&header))
    {
      return false;
    }

  if (header.magic == TINYBOOT_HEADER_MAGIC_INV)
    {
      /* Images with no precalculated CRC are considered valid. These
       * should be the images that are uploaded directly to the primary
       * paritition with debugger/flasher and are not uploaded by the
       * bootloader. These images also don't have confirmed flags,
       * altough they are considered stable.
       */

      return true;
    }
  else
    {
      return calculate_crc(fd, &header) == header.crc;
    }
}

static bool compare_versions(struct tinyboot_img_version *v1,
                             struct tinyboot_img_version *v2)
{
#ifndef CONFIG_TINYBOOT_PREVENT_DOWNGRADE
  int i;
  if (v1->major != v2->major ||
      v1->minor != v2->minor ||
      v1->patch != v2->patch)
    {
      return false;
    }

  for (i = 0; i < TINYBOOT_HEADER_PRERELEASE_MAXLEN; i++)
    {
      if (v1->pre_release[i] != v2->pre_release[i])
        {
          return false;
        }
    }

  return true;
#else
  if (v1->major > v2->major ||
      v1->minor > v2->minor ||
      v1->patch > v2->patch)
    {
      return true;
    }

  if (v1->major == v2->major &&
      v1->minor == v2->minor &&
      v1->patch == v2->patch)
    {
      /* TODO: compare prerelease */
    }

  return false;
#endif
}

static enum tinyboot_update_type
  get_update_type(int primary, int update, int recovery,
                  struct tinyboot_img_header *primary_header,
                  struct tinyboot_img_header *update_header)
{
  if (get_image_flag(recovery, TINYBOOT_CONFIRMED_PAGE_INDEX) &&
      get_image_flag(update, TINYBOOT_UPDATED_PAGE_INDEX) &&
      ((!get_image_flag(primary, TINYBOOT_CONFIRMED_PAGE_INDEX) &&
      primary_header->magic != TINYBOOT_HEADER_MAGIC_INV) ||
      !validate_image(primary)) && validate_image(recovery))
    {
      return TINYBOOT_UPDATE_TYPE_REVERT;
    }

  if (!get_image_flag(update, TINYBOOT_CONFIRMED_PAGE_INDEX) &&
      !get_image_flag(update, TINYBOOT_UPDATED_PAGE_INDEX) &&
      validate_image(update))
    {
      if (compare_versions(&primary_header->img_version,
                           &update_header->img_version) &&
          validate_image(primary))
        {
          return TINYBOOT_UPDATE_TYPE_NONE;
        }

      return TINYBOOT_UPDATE_TYPE_UPDATE;
    }

  return TINYBOOT_UPDATE_TYPE_NONE;
}

static int perform_update(struct tinyboot_state *state, bool check_only)
{
  int update;
  int recovery;
  int primary;
  int secondary;
  int tertiary;
  bool primary_valid;

  primary = flash_partition_open(CONFIG_TINYBOOT_PRIMARY_SLOT_PATH);
  assert(primary >= 0);

  secondary = flash_partition_open(CONFIG_TINYBOOT_SECONDARY_SLOT_PATH);
  assert(secondary >= 0);

  tertiary = flash_partition_open(CONFIG_TINYBOOT_TERTIARY_SLOT_PATH);
  assert(tertiary >= 0);

  if (state->update == TINYBOOT_SECONDARY_SLOT_NUM)
    {
      update = secondary;
      recovery = tertiary;
    }
  else
    {
      update = tertiary;
      recovery = secondary;
    }

  if (state->next_boot == TINYBOOT_UPDATE_TYPE_REVERT &&
      (!check_only || !validate_image(primary)))
    {
      if (validate_image(recovery))
        {
          syslog(LOG_INFO, "Reverting image to recovery.\n");
          copy_partition(recovery, primary, true, state);
        }
    }
  else
    {
      primary_valid = validate_image(primary);
      if (primary_valid && check_only)
        {
          /* Skip if primary image is valid (does not mather whether
           * confirmed or not) and check_only option is set.
           */

          goto perform_update_done;
        }

      if (!state->recovery_valid && state->primary_confirmed &&
          primary_valid)
        {
          /* Save current image as recovery only if it is valid and
           * confirmed. We have to check this in case of restart
           * during update process.
           * If board is restarted during update, primary slot contains
           * non-valid image and we do not want to copy this image to
           * recovery slot.
           * There also might be a case where primary image is valid
           * but not confirmed (timing of board reset right after
           * update is uploaded to secondary). Still we do not want
           * to upload this to recovery.
           */

          syslog(LOG_INFO, "Creating recovery image.\n");
          copy_partition(primary, recovery, false, state);
          if (!validate_image(recovery))
            {
              syslog(LOG_INFO, "New recovery is not valid, stop update\n");
              goto perform_update_done;
            }

          syslog(LOG_INFO, "Recovery image created.\n");
        }

      if (validate_image(update))
        {
          /* Perform update only if update slot contains valid image. */

          syslog(LOG_INFO, "Updating from update image.\n");
          copy_partition(update, primary, true, state);

          /* Mark update slot as updated. This is to prevent repeated
           * updates.
           */

          set_image_flag(update, TINYBOOT_UPDATED_PAGE_INDEX);
        }
    }

perform_update_done:
  flash_partition_close(primary);
  flash_partition_close(secondary);
  flash_partition_close(tertiary);
  return OK;
}

/****************************************************************************
 * Public Functions
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

int tinyboot_get_state(struct tinyboot_state *state)
{
  int primary;
  int secondary;
  int tertiary;
  int update;
  int recovery;
  struct tinyboot_img_header primary_header;
  struct tinyboot_img_header secondary_header;
  struct tinyboot_img_header tertiary_header;
  struct tinyboot_img_header *update_header;

  memset(&state->update, 0, sizeof(struct tinyboot_state) - sizeof(state->fds));

  primary = flash_partition_open(CONFIG_TINYBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  secondary = flash_partition_open(CONFIG_TINYBOOT_SECONDARY_SLOT_PATH);
  if (secondary < 0)
    {
      flash_partition_close(primary);
      return ERROR;
    }

  tertiary = flash_partition_open(CONFIG_TINYBOOT_TERTIARY_SLOT_PATH);
  if (tertiary < 0)
    {
      flash_partition_close(primary);
      flash_partition_close(secondary);
      return ERROR;
    }

  get_image_header(primary, &primary_header);
  get_image_header(secondary, &secondary_header);
  get_image_header(tertiary, &tertiary_header);

  update = secondary;
  recovery = tertiary;
  update_header = &secondary_header;
  state->update = TINYBOOT_SECONDARY_SLOT_NUM;
  state->recovery = TINYBOOT_TERTIARY_SLOT_NUM;
  if (get_image_flag(secondary, TINYBOOT_CONFIRMED_PAGE_INDEX) &&
      validate_image(secondary))
    {
      /* Secondary image is confirmed and valid, use this as
       * a potential recovery.
       */

      update = tertiary;
      recovery = secondary;
      state->recovery = TINYBOOT_SECONDARY_SLOT_NUM;
      state->update = TINYBOOT_TERTIARY_SLOT_NUM;
      update_header = &tertiary_header;

      if (secondary_header.crc == primary_header.crc)
        {
          state->recovery_valid = true;
        }
    }
  else if (get_image_flag(tertiary, TINYBOOT_CONFIRMED_PAGE_INDEX) &&
           tertiary_header.crc == primary_header.crc &&
           validate_image(tertiary))
    {
      state->recovery_valid = true;
    }

  if (get_image_flag(primary, TINYBOOT_CONFIRMED_PAGE_INDEX) ||
      primary_header.magic == TINYBOOT_HEADER_MAGIC_INV)
    {
      state->primary_confirmed = true;
    }

  state->next_boot = get_update_type(primary, update, recovery,
                                     &primary_header, update_header);

  flash_partition_close(primary);
  flash_partition_close(secondary);
  flash_partition_close(tertiary);
  return OK;
}

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

int tinyboot_get_confirm(void)
{
  int primary;
  struct tinyboot_img_header primary_header;

  primary = flash_partition_open(CONFIG_TINYBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  get_image_header(primary, &primary_header);
  if (get_image_flag(primary, TINYBOOT_CONFIRMED_PAGE_INDEX) ||
      primary_header.magic == TINYBOOT_HEADER_MAGIC_INV)
    {
      return 1;
    }

  return 0;
}

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

int tinyboot_confirm(struct tinyboot_state *state)
{
  int ret;
  int update;
  int primary;
  int secondary;
  int tertiary;
  int recovery;

  tinyboot_get_state(state);
  if (state->primary_confirmed)
    {
      return OK;
    }

  primary = flash_partition_open(CONFIG_TINYBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  secondary = flash_partition_open(CONFIG_TINYBOOT_SECONDARY_SLOT_PATH);
  if (secondary < 0)
    {
      flash_partition_close(primary);
      return ERROR;
    }

  tertiary = flash_partition_open(CONFIG_TINYBOOT_TERTIARY_SLOT_PATH);
  if (tertiary < 0)
    {
      flash_partition_close(primary);
      flash_partition_close(secondary);
      return ERROR;
    }

  if (state->update == TINYBOOT_SECONDARY_SLOT_NUM)
    {
      update = secondary;
      recovery = tertiary;
    }
  else
    {
      update = tertiary;
      recovery = secondary;
    }

  /* We need to mark both primary and update partitions as confirmed
   * (update partition will become recovery once confirmed) and
   * we have to remove confirmed flag from old recovery and set updated
   * flag there. This is to prevent old recovery still identify as
   * recovery and not to look as possible update. Therefore remove the
   * entire last sector (clears confirmed flag) and write updated
   * flag.
   */

  ret = OK;
  if (set_image_flag(primary, TINYBOOT_CONFIRMED_PAGE_INDEX) < 0)
    {
      ret = ERROR;
      goto confirm_done;
    }

  if (set_image_flag(update, TINYBOOT_CONFIRMED_PAGE_INDEX) < 0)
    {
      ret = ERROR;
      goto confirm_done;
    }

  if (flash_partition_erase_last_sector(recovery) < 0)
    {
      ret = ERROR;
      goto confirm_done;
    }

  if (set_image_flag(recovery, TINYBOOT_UPDATED_PAGE_INDEX) < 0)
    {
      ret = ERROR;
      goto confirm_done;
    }

confirm_done:
  flash_partition_close(primary);
  flash_partition_close(secondary);
  flash_partition_close(tertiary);

  return ret;
}

/****************************************************************************
 * Name: tinyboot_perform_update
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
                            struct tinyboot_state *state)
{
  int ret;

   /* All leds on */

  for (int i = 0; i < TINYBOOT_LEDS_NUM; i++)
    {
      ioctl(state->fds[i], GPIOC_WRITE, TINYBOOT_LEDS_ON);
    }

  ret = tinyboot_get_state(state);
  if (ret < 0)
    {
      return ERROR;
    }

  if (state->next_boot != TINYBOOT_UPDATE_TYPE_NONE)
    {
      /* All leds off*/

      for (int i = 0; i < TINYBOOT_LEDS_NUM; i++)
        {
          ioctl(state->fds[i], GPIOC_WRITE, TINYBOOT_LEDS_OFF);
        }

      /* We either want to update or revert. */

      ret = perform_update(state, check_only);
      if (ret < 0)
        {
          syslog(LOG_ERR, "Update process failed. %s\n",
             strerror(errno));
        }
      else
        {
          ret = tinyboot_confirm(state);
        }
    }

  return ret;
}
