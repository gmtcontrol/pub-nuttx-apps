/****************************************************************************
 * apps/gmtcnt/leds/leds_main.c
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
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/timers/pwm.h>

#include <system_status.h>
#include <network_status.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_GMTCNT_LEDS_PWMPATH	"/dev/pwm0"

#define LED_UPDATE_WAIT   (10)
#define LED_EFFECT_WAIT   (50)

#define LED_MASK_CAN     	(1)
#define LED_MASK_USB    	(2)
#define LED_MASK_232      (4)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_led_service_started = false;

static struct system_status_s  system_stat;
static struct network_status_s network_stat;
static struct pwm_info_s led_info;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_animation
 ****************************************************************************/

/**
 * Start/stop the LED animation.  This function will configure the PWM
 * output according to the information in the provided struct pwm_info_s
 * and then start or stop the pulse train.
 *
 * @param fd The file descriptor of the PWM driver to use.
 * @param start If true, start the pulse train.  Otherwise, stop the pulse
 *   train.
 * @param info If non-NULL, the struct pwm_info_s containing the PWM
 *   characteristics to use.
 *
 * @return 0 on success, a negated errno value on failure.
 */
static int led_animation(int fd, bool start, struct pwm_info_s *info)
{
  int ret;

  if (info)
    {
      /* Clear the characteristics info */

      memset(info, 0, sizeof(struct pwm_info_s));

      /* Set the PWM characteristics */

      ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS,
                  (unsigned long)((uintptr_t)info));
      if (ret < 0)
        {
          printf("led_animation: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
          return ERROR;
        }
    }

  /* Then start the pulse train.  Since the driver was opened in blocking
   * mode, this call will block if the count value is greater than zero.
   */

  if (start)
    {
      ret = ioctl(fd, PWMIOC_START, 0);
      if (ret < 0)
        {
          printf("led_animation: ioctl(PWMIOC_START) failed: %d\n", errno);
          return ERROR;
        }
    }
  else
    {
      ret = ioctl(fd, PWMIOC_STOP, 0);
      if (ret < 0)
        {
          printf("led_animation: ioctl(PWMIOC_STOP) failed: %d\n", errno);
          return ERROR;
        }
    }

  return OK;
}

/**
 * Set the LED effect.
 *
 * @param fd The file descriptor of the PWM driver.
 * @param mask A bit mask of the LEDs to set.  The least significant bit
 *   corresponds to the first channel, and so on.
 * @param duty The duty cycle of the LEDs in the mask.  0 <= duty <= 100.
 * @param update Set to true to update the PWM driver with the new
 *   characteristics.
 * @param info The struct pwm_info_s containing the PWM characteristics to
 *   use.
 *
 * @return 0 on success, a negated errno value on failure.
 */
static int led_effect(int fd, uint8_t mask, uint16_t duty, bool update, struct pwm_info_s *info)
{
  int ret, i;

  /* Update the PWM characteristics */

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      info->channels[i].channel = i + 1;
      if (mask & (1 << i))
        {
          info->channels[i].duty = duty ? b16divi(uitoub16(duty) - 1, 100) : 0;
        }
      else
        {
          info->channels[i].duty = 0;
        }
    }

  if (update)
    {
      /* Set the PWM characteristics */

      ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS,
                  (unsigned long)((uintptr_t)info));
      if (ret < 0)
        {
          printf("set_effect: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: leds_service
 ****************************************************************************/

/**
 * Main entry point for the LED daemon.
 *
 * This function is the entry point for the LED daemon.  It is responsible for
 * starting the LED daemon task.
 *
 * @param argc The number of command line arguments.
 * @param argv The command line arguments.
 *
 * @return EXIT_SUCCESS if the daemon is successfully started;
 *         EXIT_FAILURE otherwise.
 */
static int leds_service(int argc, char *argv[])
{
  int sub_network;
	int sub_system;
  bool updated;
  int fd;

  /* Indicate that we are running */

  g_led_service_started = true;
  printf("leds_service: Running\n");

  /* Open the PWM driver */

  fd = open(CONFIG_GMTCNT_LEDS_PWMPATH, O_RDONLY);
  if (fd < 0)
    {
      int errcode = errno;
      printf("leds_service: ERROR: Failed to open %s: %d\n",
             CONFIG_GMTCNT_LEDS_PWMPATH, errcode);
      goto errout;
    }

  /* Subscribe the uORB topics */

  memset(&system_stat, 0, sizeof(struct system_status_s));
  sub_system  = orb_subscribe(ORB_ID(system_status));

  memset(&network_stat, 0, sizeof(struct network_status_s));
  sub_network  = orb_subscribe(ORB_ID(network_status));

  /* Start the LED animation */

  led_animation(fd, true, &led_info);
  led_info.frequency = 200;

  uint8_t  effect_mask = LED_MASK_USB;
  uint16_t effect_wait = LED_EFFECT_WAIT;
  uint16_t update_wait = 1;
  uint32_t duty = 100;
  int8_t   step = 2;

  /* Now loop forever, changing the LED set */

  while (1)
    {
      /* LED effect wait */

      if (!effect_wait)
        {
          /* Animation logic */

          if ((duty % 100) == 0)
            {
              step *= -1;
            }
          duty += step;

          /* Update the LED effects */

          led_effect(fd, 
                     effect_mask, 
                     duty,
                     true,
                     &led_info);

          if (duty == 0)
            {
              effect_wait = LED_EFFECT_WAIT;
            }     
        }
      else
        {
          effect_wait--;
        }

      /* LED update wait */

      if (!--update_wait)
        {
          update_wait = LED_UPDATE_WAIT;
        }

      /* Check the network status topic */

      if ((orb_check(sub_network, &updated) == OK) && updated)
        {
          /* Get the published data */

          if (orb_copy(ORB_ID(network_status), sub_network, &network_stat) == OK)
            {
							/* Ethernet link status LED */

							if (network_stat.eth0.link || network_stat.eth1.link)
								{
									effect_mask |= LED_MASK_232;
								}
							else
								{
									effect_mask &= ~LED_MASK_232;
								}
            }
        }

      /* Check the system status topic */

      if ((orb_check(sub_system, &updated) == OK) && updated)
        {
          /* Get the published data */

          if (orb_copy(ORB_ID(system_status), sub_system, &system_stat) == OK)
            {
							/* FPGA status LED */

							if (system_stat.fpga_stat != 0)
								{
									effect_mask |= LED_MASK_CAN;
								}
							else
								{
									effect_mask &= ~LED_MASK_CAN;
								}
            }
        }				

      /* Wait for 10ms */

      usleep(10 * 1000L);
    }

  close(fd);

errout:
  g_led_service_started = false;

  printf("leds_service: Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * leds_main
 ****************************************************************************/

/**
 * Main entry point for the led daemon.
 *
 * This function is the entry point for the led daemon.  It is responsible for
 * starting the led daemon task.
 *
 * @param argc The number of command line arguments.
 * @param argv The command line arguments.
 *
 * @return EXIT_SUCCESS if the daemon is successfully started;
 *         EXIT_FAILURE otherwise.
 */
int main(int argc, FAR char *argv[])
{
  int ret;

  printf("leds_main: Starting the leds service\n");
  if (g_led_service_started)
    {
      printf("leds_main: leds service already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("leds_service", 
                    CONFIG_GMTCNT_LEDS_PRIORITY,
                    CONFIG_GMTCNT_LEDS_STACKSIZE, 
                    leds_service,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("leds_main: ERROR: Failed to start leds service: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("leds_main: leds service started\n");
  return EXIT_SUCCESS;
}
