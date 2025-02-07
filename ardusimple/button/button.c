/****************************************************************************
 * apps/examples/buttons/button_main.c
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

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/input/buttons.h>
#include <battery_status.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_INPUT_BUTTONS
#  error "CONFIG_INPUT_BUTTONS is not defined in the configuration"
#endif

#ifndef CONFIG_INPUT_BUTTONS_NPOLLWAITERS
#  define CONFIG_INPUT_BUTTONS_NPOLLWAITERS 2
#endif

#ifndef CONFIG_INPUT_BUTTONS_POLL_DELAY
#  define CONFIG_INPUT_BUTTONS_POLL_DELAY 1000
#endif

#define BUTTON_PRESS_CNT    (5)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_button_service_started;

static struct battery_status_s batt_stat;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_service
 ****************************************************************************/

static int button_service(int argc, char *argv[])
{
  btn_buttonset_t sample = 0;
  int fd, ret, counter = 0;
  struct pollfd fds[1];
  bool isTurnedOn = false;

  /* Indicate that we are running */

  g_button_service_started = true;
  printf("button_service: Running\n");

  /* Open the BUTTON driver */

  fd = open(CONFIG_ARDUSIMPLE_BUTTON_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      int errcode = errno;
      printf("button_service: ERROR: Failed to open %s: %d\n",
             CONFIG_ARDUSIMPLE_BUTTON_DEVPATH, errcode);
      goto errout;
    }

  /* Subscribe the battery service */

  memset(&batt_stat, 0, sizeof(batt_stat));

  int batt_sub = orb_subscribe(ORB_ID(battery_status));
  bool uorb_started = false;

  /* Now loop forever, waiting BUTTONs events */

  while (1)
    {
      bool updated;
      bool timeout;
      int  nbytes;

      /* Check the uORB status */

      if ((orb_check(batt_sub, &updated) == OK) && updated)
        {
          /* Get the published data */

          if (orb_copy(ORB_ID(battery_status), batt_sub, &batt_stat) == OK)
            {
              /* Set the flag to inform data has been collected */

              uorb_started = true;
            }
        }

      /* Check the USB cable */

      if ((uorb_started == FALSE) || batt_stat.cable)
        {
          /* Wait for a 10ms */

          usleep(10 * 1000L);
          continue;
        }

      /* Prepare the File Descriptor for poll */

      memset(fds, 0, sizeof(fds));
      fds[0].fd     = fd;
      fds[0].events = POLLIN;
      timeout       = false;

      /* Poll the button */

      ret = poll(fds, 1, CONFIG_INPUT_BUTTONS_POLL_DELAY);
      if (ret < 0)
        {
          int errcode = errno;
          printf("button_service: ERROR poll failed: %d\n", errcode);
        }
      else if (ret == 0)
        {
          printf("button_service: Timeout\n");
          timeout = true;
        }
      else if (ret > CONFIG_INPUT_BUTTONS_NPOLLWAITERS)
        {
          printf("button_service: ERROR poll reported: %d\n", errno);
        }

      /* In any event, read until the pipe is empty */

      do
        {
          nbytes = read(fds[0].fd, (void *)&sample, sizeof(btn_buttonset_t));

          if (nbytes <= 0)
            {
              if (nbytes == 0 || errno == EAGAIN)
                {
                  if ((fds[0].revents & POLLIN) != 0)
                    {
                      printf("button_service: ERROR no read data\n");
                    }
                }
              else if (errno != EINTR)
                {
                  printf("button_service: read failed: %d\n", errno);
                }

              nbytes = 0;
            }
          else
            {
              if (timeout)
                {
                  printf("button_service: ERROR Poll timeout, "
                         "but data read\n");
                  printf("               (might just be a race "
                         "condition)\n");
                }
            }

          /* Suppress error report if no read data on the next time
           * through
           */

          fds[0].revents = 0;

          /* Check the sample value */

          if (sample != 0)
            {
              /* Increment the counter */

              if (++counter > BUTTON_PRESS_CNT)
                {
                  /* Check the action */

                  if (isTurnedOn)
                    {
#ifdef CONFIG_SYSTEM_SYSTEM
                      system("tone -x");
                      system("poweroff");
#endif /* CONFIG_SYSTEM_SYSTEM */
                    }
                    else
                      {
#ifdef CONFIG_SYSTEM_SYSTEM
                        system("tone -s");
#endif /* CONFIG_SYSTEM_SYSTEM */
                        isTurnedOn = true;
                      }
                }
            }
          else
            {
              /* Wait for button release to poweroff */

              if (!isTurnedOn)
                {
                  usleep(1000 * 1000L);

                  /* Power off the device */

#ifdef CONFIG_SYSTEM_SYSTEM
                  system("poweroff");
#endif /* CONFIG_SYSTEM_SYSTEM */
                }

              /* Clear the counter */

              counter = 0;
            }

          /* Wait for a 500ms */

          usleep(500 * 1000L);
        }
      while (nbytes > 0);
    }

  close(fd);

errout:
  g_button_service_started = false;

  printf("button_service: Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * button_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("button_main: Starting the button service\n");
  if (g_button_service_started)
    {
      printf("button_main: button service already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("button_service",
                    CONFIG_ARDUSIMPLE_BUTTON_PRIORITY,
                    CONFIG_ARDUSIMPLE_BUTTON_STACKSIZE,
                    button_service,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("button_main: ERROR: Failed to start button service: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("button_main: button service started\n");
  return EXIT_SUCCESS;
}
