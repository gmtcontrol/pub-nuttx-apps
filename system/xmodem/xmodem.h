/****************************************************************************
 * apps/system/xmodem/xmodem.h
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

#ifndef __APPS_SYSTEM_XMODEM_XMODEM_H
#define __APPS_SYSTEM_XMODEM_XMODEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XMODEM_PACKET_HEADER      (3)
#define XMODEM_PACKET_TRAILER     (3)
#define XMODEM_PACKET_OVERHEAD    (XMODEM_PACKET_HEADER + XMODEM_PACKET_TRAILER)
#define XMODEM_PACKET_SIZE        (128)
#define XMODEM_PACKET_1K_SIZE     (1024)

#define XMODEM_FILENAME_PACKET    (0)
#define XMODEM_DATA_PACKET        (1)

#define FILE_NAME_LENGTH          (256)
#define FILE_SIZE_LENGTH          (16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct xmodem_priv_s
{
  int fd;
  FAR char *file_name;
  size_t file_saved_size;
};

struct xmodem_ctx_s
{
  /* User need initialization */

  FAR void *priv;
  int recvfd;
  int sendfd;
  int retrans;
  int retry;
  int packet_no;

  /* Public data */

  size_t packet_size;
  int packet_type;
  char file_name[PATH_MAX];
  size_t file_length;

  /* Private data */

  FAR uint8_t *header;
#ifdef CONFIG_SYSTEM_XMODEM_DEBUG_FILEPATH
  int debug_fd;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int xmodem_recv(FAR struct xmodem_ctx_s *ctx);
int xmodem_send(FAR struct xmodem_ctx_s *ctx);

#endif /* __APPS_SYSTEM_XMODEM_XMODEM_H */
