/****************************************************************************
 * apps/ardusimple/msend/msend.c
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

#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msend_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  char buffer[64];
  ssize_t nbytes;
  int ret, count;
  int fds = -1;
  int exitcode = 0;

  /* Open FIFOs */

  printf("\nmsend_main: Creating FIFO %s\n", CONFIG_ARDUSIMPLE_MSEND_FIFO);
  ret = mkfifo(CONFIG_ARDUSIMPLE_MSEND_FIFO, 0666);
  if (ret < 0)
    {
      if (EEXIST != errno)
        {
          printf("msend_main: mkfifo failed: %d\n", errno);
          exitcode = 1;
          goto errout;
        }
    }

  /* Open the FIFOs for nonblocking, write */

  fds = open(CONFIG_ARDUSIMPLE_MSEND_FIFO, O_WRONLY | O_NONBLOCK);
  if (fds < 0)
    {
      printf("msend_main: Failed to open FIFO %s for writing, errno=%d\n",
             CONFIG_ARDUSIMPLE_MSEND_FIFO, errno);
      exitcode = 2;
      goto errout;
    }

  /* Loop forever */

  for (count = 0; ; count++)
    {
      /* Send a message to the listener... this should wake the listener
       * from the poll.
       */

      snprintf(buffer, sizeof(buffer), "Message %d", count);
      nbytes = write(fds, buffer, strlen(buffer));
      if (nbytes < 0)
        {
          printf("msend_main: Write to fds failed: %d\n", errno);
          exitcode = 3;
          goto errout;
        }
      else
        {
          printf("msend_main: Sent '%s' (%ld bytes)\n",
                buffer, (long)nbytes);
        }

      fflush(stdout);

      /* Wait awhile.  This delay should be long enough that the
       * listener will timeout.
       */

      sleep(1);
    }

errout:
  if (fds >= 0)
    {
      close(fds);
    }

  fflush(stdout);
  return exitcode;
}
