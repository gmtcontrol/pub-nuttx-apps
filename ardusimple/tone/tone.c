/****************************************************************************
 * apps/ardusimple/tone/tone.c
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/timers/pwm.h>

#include "tone.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * play_tone
 ****************************************************************************/

static int play_tone(int fd, uint32_t freq, uint16_t duty, uint32_t on_time, uint32_t off_time)
{
  struct pwm_info_s info;
  int ret;

  /* Prepare the characteristics info */

  memset(&info, 0, sizeof(info));
  info.frequency = freq;
#ifdef CONFIG_PWM_MULTICHAN
  info.channels[0].channel = 1;
  info.channels[0].duty  = duty ? b16divi(uitoub16(duty) - 1, 100) : 0;
  info.channels[0].cpol  = 0;
  info.channels[0].dcpol = 0;
#else
  info.duty = duty ? b16divi(uitoub16(duty) - 1, 100) : 0;
#endif

  /* Set the PWM characteristics */

  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("play_tone: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      return ERROR;
    }

  /* Then start the pulse train.  Since the driver was opened in blocking
   * mode, this call will block if the count value is greater than zero.
   */

  ret = ioctl(fd, PWMIOC_START, 0);
  if (ret < 0)
    {
      printf("play_tone: ioctl(PWMIOC_START) failed: %d\n", errno);
      return ERROR;
    }

  /* Wait for the specified duration (sound on) */

  usleep(on_time * 1000);

  /* Then stop the pulse train */

  ret = ioctl(fd, PWMIOC_STOP, 0);
  if (ret < 0)
    {
      printf("play_tone: ioctl(PWMIOC_STOP) failed: %d\n", errno);
      return ERROR;
    }

  /* Wait for the specified duration (sound off) */

  usleep(off_time * 1000);

  return OK;
}

/****************************************************************************
 * play_startup_tone
 ****************************************************************************/

int play_startup_tone(int fd)
{
  const int loop_cnt = 3;

  /* Something is playing */

  for (int i = 0; i < loop_cnt; i++)
    {
      if (play_tone(fd, 500, 50, 100, 50) != OK)
        {
          return ERROR;
        }
    }

  return play_tone(fd, 2000, 50, 500, 0);
}

/****************************************************************************
 * play_shutdown_tone
 ****************************************************************************/

int play_shutdown_tone(int fd)
{
  const int loop_cnt = 10;
  int freq = 2000;

  /* Something is playing */

  for (int i = 0; i < loop_cnt; i++, freq -= freq/loop_cnt)
    {
      if (play_tone(fd, freq, 50, 100, 20) != OK)
        {
          return ERROR;
        }
    }

  return play_tone(fd, 0, 0, 0, 0);
}


/****************************************************************************
 * Name: tone_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd, opt;

  /* Open the PWM device for reading */

  fd = open(CONFIG_ARDUSIMPLE_TONE_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("tone_main: open %s failed: %d\n", CONFIG_ARDUSIMPLE_TONE_DEVPATH, errno);
      goto errout;
    }

  /* Get the command line parameters */

  while ((opt = getopt(argc, argv, "sx")) != EOF)
    {
      switch (opt)
        {
          case 's':
            play_startup_tone(fd);
            break;

          case 'x':
            play_shutdown_tone(fd);
            break;

          default:
            return 1;
        }
    }

  /* Close the device */

  close(fd);
  fflush(stdout);
  return OK;

errout:
  fflush(stdout);
  return ERROR;
}
