/****************************************************************************
 * apps/system/xmodem/rx_main.c
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

#include <fcntl.h>
#include <stdio.h>
#include <getopt.h>
#include <pthread.h>

#include "xmodem.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  printf("USAGE: %s [OPTIONS]\n", progname);
  printf("\nWhere:\n");
  printf("\nand OPTIONS include the following:\n");
  printf("\t-d <device>  : Communication device to use. Default: stdin & stdout\n");
  printf("\t-f <file_name>: Save remote file_name.\n");

  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct xmodem_priv_s priv;
  struct xmodem_ctx_s ctx;
  FAR char *dev_name = NULL;
  int ret;

  memset(&priv, 0, sizeof(priv));
  memset(&ctx, 0, sizeof(ctx));
  while ((ret = getopt(argc, argv, "d:f:h")) != ERROR)
    {
      switch (ret)
        {
          case 'd':
            dev_name = optarg;
            break;

          case 'f':
            priv.file_name = optarg;
            if (priv.file_name[strlen(priv.file_name)] == '/')
              {
                priv.file_name[strlen(priv.file_name)] = '\0';
              }
            break;

          case 'h':
            show_usage(argv[0]);
            break;

          case '?':
          default:
            show_usage(argv[0]);
            break;
        }
    }

  /* assign the private data */

  ctx.priv = &priv;

  /* open the received file */

  if (priv.file_name)
    {
      priv.fd = open(priv.file_name, O_CREAT | O_WRONLY, 0777);

      if (priv.fd < 0)
        {
          printf("ERROR: can't open %s\n", priv.file_name);
          goto out;
        }
    }

  /* open the communication device */

  if (dev_name)
    {
      ctx.recvfd = open(dev_name, O_RDWR | O_NONBLOCK);
      if (ctx.recvfd < 0)
        {
          printf("ERROR: can't open %s\n", dev_name);
          goto out;
        }

        ctx.sendfd = ctx.recvfd;
    }
  else
    {
      ctx.recvfd = STDIN_FILENO;
      ctx.sendfd = STDOUT_FILENO;
    }

  ret = xmodem_recv(&ctx);

  /* file received from remote */

  if (ret > 0)
    {
      printf("\r\n SUCCESS: %d bytes received\n", ret);
    }
  else
    {
      /* remove the uncompleted file */

      unlink(priv.file_name);
    }

  /* close the open files */

  if (ctx.recvfd > 0)
    {
      close(ctx.recvfd);
    }

out:
  if (priv.fd > 0)
    {
      close(priv.fd);
    }

  return ret;
}
