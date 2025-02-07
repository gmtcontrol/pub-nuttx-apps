/****************************************************************************
 * apps/imarine/adc/adc_read.c
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
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include "queue.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ITEM_SIZE   (sizeof(int32_t)*4)
#define DQUE_SIZE   (16384)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_adc_daemon_started;
static uint8_t dq_buf[ITEM_SIZE * DQUE_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigterm_action
 ****************************************************************************/

static void sigterm_action(int signo, siginfo_t *siginfo, void *arg)
{
  if (signo == SIGTERM)
    {
      printf("SIGTERM received\n");

      g_adc_daemon_started = false;
      printf("adc_daemon: Terminated.\n");
    }
  else
    {
      printf("\nsigterm_action: Received signo=%d siginfo=%p arg=%p\n",
             signo, siginfo, arg);
    }
}

/****************************************************************************
 * Name: adc_daemon
 ****************************************************************************/

static int adc_daemon(int argc, char *argv[])
{
  FAR volatile int32_t *chA_data = (volatile int32_t *)(CONFIG_IMARINE_ADC_MEMADDR + 0x10);
  FAR volatile int32_t *chB_data = (volatile int32_t *)(CONFIG_IMARINE_ADC_MEMADDR + 0x20);
  struct sigaction act;
  int32_t dbuf[4];
  pid_t mypid;
  queue_t dq;
  int ret;

  /* Create the data queue*/

  QCreate(&dq, DQUE_SIZE, ITEM_SIZE, dq_buf);

  /* SIGTERM handler */

  memset(&act, 0, sizeof(struct sigaction));
  act.sa_sigaction = sigterm_action;
  act.sa_flags     = SA_SIGINFO;

  sigemptyset(&act.sa_mask);
  sigaddset(&act.sa_mask, SIGTERM);

  ret = sigaction(SIGTERM, &act, NULL);
  if (ret != 0)
    {
      fprintf(stderr, "Failed to install SIGTERM handler, errno=%d\n",
              errno);
      return (EXIT_FAILURE + 1);
    }

  /* Indicate that we are running */

  mypid = getpid();
  g_adc_daemon_started = true;
  printf("\nadc_daemon (pid# %d): Running\n", mypid);

  /* Send the ADC driver data */

  while (g_adc_daemon_started == true)
    {
      do
      {
        /* get the ADC data */

        dbuf[0] = chA_data[0];
        dbuf[1] = chA_data[1];
        dbuf[2] = chB_data[0];
        dbuf[3] = chB_data[1];

        /* put all of them to the queue */

        /* do it till buffer is full */

      } while (QEnqueue(&dq, dbuf));

      /* consume the data queue */

      while (QDequeue(&dq, dbuf))
        {
          printf("$:%d:%d:%d:%d:\n", dbuf[0], dbuf[1], dbuf[2], dbuf[3]);
          //printf("$:%d:%d:%d:%d:\n", chA_data[0], chA_data[1], chB_data[0], chB_data[1]);
        }

        /* make sure the buffer is empty */

        QFlush(&dq);
    }

  /* treats signal termination of the task
   * task terminated by a SIGTERM
   */

  exit(EXIT_SUCCESS);

  g_adc_daemon_started = false;
  printf("adc_daemon: Terminating\n");

  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * adc_read
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("adc_read: Starting the adc_daemon\n");
  if (g_adc_daemon_started)
    {
      printf("adc_read: adc_daemon already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("adc_daemon",
                    CONFIG_IMARINE_ADC_PRIORITY,
                    CONFIG_IMARINE_ADC_STACKSIZE,
                    adc_daemon,
                    argv);
  if (ret < 0)
    {
      int errcode = errno;
      printf("adc_read: ERROR: Failed to start adc_daemon: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("adc_read: adc_daemon started\n");
  return EXIT_SUCCESS;
}
