/****************************************************************************
 * apps/ardusimple/mpoll/mpoll.c
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
#define NPOLLFDS    1
#define FIFONDX     0

#define POLL_LISTENER_DELAY   2000   /* 2 seconds */

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
 * Name: mpoll_main
 ****************************************************************************/

static void *mpoll_main(pthread_addr_t pvarg)
{
  static char buffer[CONFIG_DEV_FIFO_SIZE];
  struct pollfd fds[NPOLLFDS];
  ssize_t nbytes;
  bool timeout;
  bool pollin;
  int nevents;
  int fd;
  int ret;
  int i;

  /* Open the FIFO for non-blocking read */

  printf("mpoll_main: Opening %s for non-blocking read\n", CONFIG_ARDUSIMPLE_MPOLL_FIFO);

  fd = open(CONFIG_ARDUSIMPLE_MPOLL_FIFO, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      printf("mpoll_main: ERROR Failed to open FIFO %s: %d\n",
             CONFIG_ARDUSIMPLE_MPOLL_FIFO, errno);
      close(fd);
      return (FAR void *)-1;
    }

  /* Loop forever */

  while (1)
    {
      memset(fds, 0, sizeof(struct pollfd)*NPOLLFDS);
      fds[FIFONDX].fd      = fd;
      fds[FIFONDX].events  = POLLIN;
      fds[FIFONDX].revents = 0;

      timeout = false;
      pollin  = false;

      /* poll the FIFO */

      ret = poll(fds, NPOLLFDS, POLL_LISTENER_DELAY);

      if (ret < 0)
        {
          printf("mpoll_main: ERROR poll failed: %d\n", errno);
        }
      else if (ret == 0)
        {
          printf("mpoll_main: Timeout\n");
          timeout = true;
        }
      else if (ret > NPOLLFDS)
        {
          printf("mpoll_main: ERROR poll reported: %d\n", errno);
        }
      else
        {
          pollin = true;
        }

      nevents = 0;
      for (i = 0; i < NPOLLFDS; i++)
        {
          if (timeout)
            {
              if (fds[i].revents != 0)
                {
                  printf("mpoll_main: ERROR expected revents=00, "
                         "received revents[%d]=%08" PRIx32 "\n",
                         i, fds[i].revents);
                }
            }
          else if (pollin)
            {
              if (fds[i].revents == POLLIN)
                {
                  nevents++;
                }
              else if (fds[i].revents != 0)
                {

                }
            }
        }

      if (pollin && nevents != ret)
        {

        }

      /* In any event, read until the pipe/serial  is empty */

      for (i = 0; i < NPOLLFDS; i++)
        {
          do
            {
              /* The pipe works differently, it returns whatever data
                * it has available without blocking.
                */

              nbytes = read(fds[i].fd, buffer, sizeof(buffer)-1);

              if (nbytes <= 0)
                {
                  if (nbytes == 0 || errno == EAGAIN)
                    {
                      if ((fds[i].revents & POLLIN) != 0)
                        {

                        }
                    }
                  else if (errno != EINTR)
                    {

                    }

                  nbytes = 0;
                }
              else
                {
                  if (timeout)
                    {

                    }

                  buffer[nbytes] = '\0';
                  printf("mpoll_main: Read[%d] '%s' (%ld bytes)\n",
                         i, buffer, (long)nbytes);
                }

              /* Suppress error report if no read data on the next
               * time through
               */

              fds[i].revents = 0;
            }
          while (nbytes > 0);
        }

      /* Make sure that everything is displayed */

      fflush(stdout);
    }

  /* Won't get here */

  close(fd);
  return NULL;
}

/****************************************************************************
 * Name: mpoll_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int exitcode = 0;
  pthread_t tid;
  int ret;

  /* Open FIFOs */

  printf("\nmpoll_main: Creating FIFO %s\n", CONFIG_ARDUSIMPLE_MPOLL_FIFO);
  ret = mkfifo(CONFIG_ARDUSIMPLE_MPOLL_FIFO, 0666);
  if (ret < 0)
    {
      if (EEXIST != errno)
        {
          printf("mpoll_main: mkfifo failed: %d\n", errno);
          exitcode = 1;
          goto errout;
        }
    }

  /* Start the listeners */

  printf("mpoll_main: Starting listener thread\n");

  ret = pthread_create(&tid, NULL, mpoll_main, NULL);
  if (ret != 0)
    {
      printf("mpoll_main: Failed to create listener thread: %d\n", ret);
      exitcode = 2;
      goto errout;
    }

  /* Loop forever */

  while (1)
    {
      sleep(5);
    }

errout:
  fflush(stdout);
  return exitcode;
}
