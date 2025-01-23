/****************************************************************************
 * apps/examples/mongoose/smart/device.c
 *
 *   Copyright (C) 2007, 2009-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Copyright (c) 2001, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Adam Dunkels.
 * 4. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <debug.h>

#include <mongoose.h>

#ifdef CONFIG_LIBFLASH
#  include <flash/partition.h>
#else
#  warning "Firmware upgrade trusts the Flash library support!"
#endif /* CONFIG_LIBFLASH */

#ifdef CONFIG_BOARDCTL_RESET
#  include <sys/boardctl.h>
#  include <sys/ioctl.h>
#endif

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_MONGOOSE_SYSTEM_OTAPATH
#  define CONFIG_EXAMPLES_MONGOOSE_SYSTEM_OTAPATH   "/dev/img0"
#endif

/****************************************************************************
 * Private Type
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Global OTA file descriptor */

int g_ota_fd = -1;

/****************************************************************************
 * Mongoose Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mg_device_reset
 ****************************************************************************/

void mg_device_reset(void)
{
#ifdef CONFIG_BOARDCTL_RESET
  boardctl(BOARDIOC_RESET, 0);
#endif
}

/****************************************************************************
 * Name: mg_ota_begin
 ****************************************************************************/

bool mg_ota_begin(size_t new_firmware_size)
{
  bool ret = false;

  UNUSED(new_firmware_size);

  /* Open the partition to save firmware */

  g_ota_fd = open(CONFIG_EXAMPLES_MONGOOSE_SYSTEM_OTAPATH,
                  O_CREAT | O_WRONLY, 0777);
  if (g_ota_fd >= 0)
    {
      /* Erase last sector of the partition to inform file is new */

#ifdef CONFIG_LIBFLASH
      if (flash_partition_erase_last_sector(g_ota_fd) < 0)
        {
          close(g_ota_fd);
          g_ota_fd = -1;
          return false;
        }
#endif /* CONFIG_LIBFLASH */

        ret = true;
    }

  return ret;
}

/****************************************************************************
 * Name: mg_ota_write
 ****************************************************************************/

bool mg_ota_write(const void *buf, size_t len)
{
  if (g_ota_fd >= 0)
    {
      if (write(g_ota_fd, buf, len) == len)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: mg_ota_end
 ****************************************************************************/

bool mg_ota_end(void)
{
  bool ret = false;

  if (g_ota_fd > 0)
    {
      if (close(g_ota_fd) == OK)
        {
          ret = true;
        }
      g_ota_fd = -1;
    }

  return ret;
}

/****************************************************************************
 * Name: mg_ota_size
 ****************************************************************************/

size_t mg_ota_size(int fw)
{
  int ret = 0;
#ifdef CONFIG_LIBFLASH
  struct flash_partition_info info;
  int fd = g_ota_fd;

  if (fd < 0)
    {
      fd = open(CONFIG_EXAMPLES_MONGOOSE_SYSTEM_OTAPATH,
                O_RDONLY, 0777);
    }

  if (flash_partition_info(fd, &info) > 0)
    {
      ret = info.size;
    }

  if (g_ota_fd < 0)
    {
      close(fd);
    }
#endif /* CONFIG_LIBFLASH */

  return ret;
}

/****************************************************************************
 * Name: mg_ota_commit
 ****************************************************************************/

bool mg_ota_commit(void)
{
  return true;
}

/****************************************************************************
 * Name: mg_ota_rollback
 ****************************************************************************/

bool mg_ota_rollback(void)
{
  return true;
}

/****************************************************************************
 * Name: mg_ota_status
 ****************************************************************************/

int mg_ota_status(int fw)
{
  return 0;
}

/****************************************************************************
 * Name: mg_ota_crc32
 ****************************************************************************/

uint32_t mg_ota_crc32(int fw)
{
  return 0;
}

/****************************************************************************
 * Name: mg_ota_timestamp
 ****************************************************************************/

uint32_t mg_ota_timestamp(int fw)
{
  return 0;
}
