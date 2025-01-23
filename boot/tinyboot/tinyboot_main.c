/****************************************************************************
 * apps/boot/tinyboot/tinyboot_main.c
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

#include <sys/boardctl.h>
#include <stdio.h>
#include <syslog.h>

#include <tinyboot.h>

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern int nsh_main(int argc, FAR char *argv[]);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct tinyboot_state g_boot_state;

/****************************************************************************
 * Name: tinyboot_thread
 ****************************************************************************/

static int tinyboot_thread(int argc, FAR char** argv)
{
  struct tinyboot_state *state = &g_boot_state;
  uint8_t leds = 0;
  UNUSED(argc);
  UNUSED(argv);

  while (true)
    {
      /* Toogle LED(s)*/

      leds ^= 1;

      /* Update all LED(s) */

      for (int i = 0; i < TINYBOOT_LEDS_NUM; i++)
        {
          if (state->fds[i] >= 0)
            {
              ioctl(state->fds[i], GPIOC_WRITE, leds);
            }
        }

      /* Suspend 200 ms */

      usleep(200 * 1000);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tinyboot_main
 *
 * Description:
 *   Tiny bootlaoder entry point.
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct boardioc_boot_info_s info;
  bool check_only;
#ifdef CONFIG_TINYBOOT_SWRESET_ONLY
  FAR struct boardioc_reset_cause_s cause;
#endif
  int ret;

#if defined(CONFIG_BOARDCTL) && !defined(CONFIG_NSH_ARCHINIT)
  /* Perform architecture-specific initialization (if configured) */

  boardctl(BOARDIOC_INIT, 0);

#ifdef CONFIG_BOARDCTL_FINALINIT
  /* Perform architecture-specific final-initialization (if configured) */

  boardctl(BOARDIOC_FINALINIT, 0);
#endif
#endif

  syslog(LOG_INFO, "*** tinyboot ***\n");

#ifdef CONFIG_TINYBOOT_SWRESET_ONLY
  check_only = true;
  ret = boardctl(BOARDIOC_RESET_CAUSE, (uintptr_t)&cause);
  if (ret >= 0)
    {
      if (cause.cause == BOARDIOC_RESETCAUSE_CPU_SOFT ||
          cause.cause == BOARDIOC_RESETCAUSE_CPU_RWDT ||
          cause.cause == BOARDIOC_RESETCAUSE_PIN)
        {
          check_only = false;
        }
      else
        {
          syslog(LOG_INFO, "Power reset detected, performing check only.\n");
        }
    }
#else
  check_only = false;
#endif

  /* open the LED(s) */

  memset(&g_boot_state, 0, sizeof(struct tinyboot_state));
  g_boot_state.fds[0] = open("/dev/gpio2", O_WRONLY);
  g_boot_state.fds[1] = open("/dev/gpio3", O_WRONLY);
  g_boot_state.fds[2] = open("/dev/gpio4", O_WRONLY);

  /* Check the partitions and perform the update */

  if (tinyboot_perform_update(check_only, &g_boot_state) < 0)
    {
      syslog(LOG_ERR, "Could not find bootable image.\n");

      /* close all LED(s) */

      for (int i = 0; i < TINYBOOT_LEDS_NUM; i++)
        {
          if (g_boot_state.fds[i] >= 0)
            {
              ioctl(g_boot_state.fds[i], GPIOC_WRITE, TINYBOOT_LEDS_OFF);
              close(g_boot_state.fds[i]);
            }
        }

      return 0;
    }

  /* Check the primary image confirmation */

  if (tinyboot_get_confirm())
    {
      syslog(LOG_INFO, "Found bootable image, boot from primary.\n");

      /* close all LED(s) */

      for (int i = 0; i < TINYBOOT_LEDS_NUM; i++)
        {
          if (g_boot_state.fds[i] >= 0)
            {
              ioctl(g_boot_state.fds[i], GPIOC_WRITE, TINYBOOT_LEDS_OFF);
              close(g_boot_state.fds[i]);
            }
        }

      /* Call board specific image boot */

      info.path        = CONFIG_TINYBOOT_PRIMARY_SLOT_PATH;
      info.header_size = CONFIG_TINYBOOT_HEADER_SIZE;

      return boardctl(BOARDIOC_BOOT_IMAGE, (uintptr_t)&info);
    }
  else
    {
      /* Create the background task */

      ret = task_create("tinyboot_service",
                        SCHED_PRIORITY_DEFAULT,
                        CONFIG_DEFAULT_TASK_STACKSIZE,
                        tinyboot_thread,
                        NULL);
      if (ret < 0)
        {
          return EXIT_FAILURE;
        }

      /* Call the shell application */

      return nsh_main(argc, argv);
    }
}
