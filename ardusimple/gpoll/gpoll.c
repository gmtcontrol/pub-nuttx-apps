/****************************************************************************
 * apps/ardusimple/gpoll/gpoll.c
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
#define GPSFIFODX   0

#define POLL_LISTENER_DELAY   1000   /* 1 seconds */

#define MINMEA_MAX_LENGTH     128

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
 * Name: gpoll_main
 ****************************************************************************/

static void *gpoll_main(pthread_addr_t pvarg)
{
  static char buffer[CONFIG_DEV_FIFO_SIZE];
  struct pollfd fds[NPOLLFDS];
  char line[MINMEA_MAX_LENGTH];
  int i, fd, cnt, ret;
  ssize_t nbytes;
  bool timeout;
  bool pollin;
  int nevents;
  char ch;

  /* Open the FIFO for non-blocking read */

  printf("gpoll_main: Opening %s for non-blocking read\n", CONFIG_ARDUSIMPLE_GPOLL_FIFO);

  fd = open(CONFIG_ARDUSIMPLE_GPOLL_FIFO, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      printf("gpoll_main: ERROR Failed to open FIFO %s: %d\n",
             CONFIG_ARDUSIMPLE_GPOLL_FIFO, errno);
      close(fd);
      return (FAR void *)-1;
    }

  /* Loop forever */

  while (1)
    {
      memset(fds, 0, sizeof(struct pollfd)*NPOLLFDS);
      fds[GPSFIFODX].fd      = fd;
      fds[GPSFIFODX].events  = POLLIN;
      fds[GPSFIFODX].revents = 0;

      timeout = false;
      pollin  = false;

      /* poll the FIFO */

      ret = poll(fds, NPOLLFDS, POLL_LISTENER_DELAY);

      if (ret < 0)
        {
          printf("gpoll_main: ERROR poll failed: %d\n", errno);
        }
      else if (ret == 0)
        {
          printf("gpoll_main: Timeout\n");
          timeout = true;
        }
      else if (ret > NPOLLFDS)
        {
          printf("gpoll_main: ERROR poll reported: %d\n", errno);
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
                  printf("gpoll_main: ERROR expected revents=00, "
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
                  /*
                  printf("gpoll_main: ERROR unexpected revents[%d]="
                         "%08" PRIx32 "\n", i, fds[i].revents);
                  */
                }
            }
        }

      if (pollin && nevents != ret)
        {
          /*
          printf("gpoll_main: ERROR found %d events, "
                  "poll reported %d\n", nevents, ret);
          */
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
                          printf("gpoll_main: ERROR no read"
                                 " data[%d]\n", i);
                        }
                    }
                  else if (errno != EINTR)
                    {
                      printf("gpoll_main: read[%d] failed: %d\n",
                             i, errno);
                    }

                  nbytes = 0;
                }
              else
                {
                  if (timeout)
                    {
                      printf("gpoll_main: ERROR? Poll timeout, "
                              "but data read[%d]\n", i);
                      printf("               (might just be "
                             "a race condition)\n");
                    }

                  /* Read until we complete a line */
                  cnt = 0;
                  for (int j = 0; j < nbytes; j++)
                    {
                      ch = buffer[j];
                      if ((ch != '\n') && (cnt < MINMEA_MAX_LENGTH))
                        {
                          line[cnt++] = ch;
                        }

                      if ((ch == '\n') && (cnt > 1))
                        {
                          line[cnt-1] = '\0';
                          printf("gpoll_main: Read[%d] '%s' (%ld bytes)\n", i, line, (long)cnt-1);

                          cnt = 0;
                        }
                    }
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
 * Name: gpoll_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int exitcode = 0;
  pthread_t tid;
  int ret;

  /* Open FIFOs */

  printf("\ngpoll_main: Creating FIFO %s\n", CONFIG_ARDUSIMPLE_GPOLL_FIFO);
  ret = mkfifo(CONFIG_ARDUSIMPLE_GPOLL_FIFO, 0666);
  if (ret < 0)
    {
      if (EEXIST != errno)
        {
          printf("gpoll_main: mkfifo failed: %d\n", errno);
          exitcode = 1;
          goto errout;
        }
    }

  /* Start the listeners */

  printf("gpoll_main: Starting listener thread\n");

  ret = pthread_create(&tid, NULL, gpoll_main, NULL);
  if (ret != 0)
    {
      printf("gpoll_main: Failed to create listener thread: %d\n", ret);
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
