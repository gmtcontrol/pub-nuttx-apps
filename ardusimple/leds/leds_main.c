/****************************************************************************
 * apps/ardusimple/leds/leds_main.c
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

#include <nuttx/leds/userled.h>
#include <nuttx/timers/pwm.h>

#include <battery_status.h>
#include <network_status.h>
#include <btstack_status.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED_UPDATE_WAIT   (10)
#define LED_EFFECT_WAIT   (50)

#define LED_MASK_BLUE     (1)
#define LED_MASK_GREEN    (2)
#define LED_MASK_RED      (4)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_led_service_started;

static struct battery_status_s battery_stat;
static struct network_status_s network_stat;
static struct btstack_status_s btstack_stat;
static struct pwm_info_s led_info;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_animation
 ****************************************************************************/

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

/****************************************************************************
 * Name: led_effect
 *
 * Color    Meaning
 * -----    -----------------------
 * Red      No Wifi, No BT SPP
 * Yellow   WiFi Connected
 * Magenta  BT SPP Connected
 * White 	  WiFi & BT SPP Connected
 ****************************************************************************/

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
 * Name: led_logic
 ****************************************************************************/

static int led_logic(int percentage, bool cable, int state)
{
  static bool blink = false;
  int led = 0x00;

  if (percentage < 10)
    {
      led = 0x04;
    }
  else if(percentage < 30)
    {
      led = 0x06;
    }
  else
    {
      led = 0x02;
    }

  if (blink && (cable == true) && (state != 3))
    {
      led = 0x00;
      blink = false;
    }
  else
    {
      blink = true;
    }

  return led;
}

/****************************************************************************
 * Name: leds_service
 ****************************************************************************/

static int leds_service(int argc, char *argv[])
{
  userled_set_t supported=0;
  userled_set_t ledset=0;
  int fd_led, fd_pwm;
  bool uorb_started;
  int sub_battery;
  int sub_network;
  int sub_btstack;
  bool updated;
  int ret;

  /* Indicate that we are running */

  g_led_service_started = true;
  printf("leds_service: Running\n");

  /* Open the PWM driver */

  fd_pwm = open(CONFIG_ARDUSIMPLE_LEDS_PWMPATH, O_RDONLY);
  if (fd_pwm < 0)
    {
      int errcode = errno;
      printf("leds_service: ERROR: Failed to open %s: %d\n",
             CONFIG_ARDUSIMPLE_LEDS_PWMPATH, errcode);
      goto errout;
    }

  /* Open the LED driver */

  fd_led = open(CONFIG_ARDUSIMPLE_LEDS_DEVPATH, O_WRONLY);
  if (fd_led < 0)
    {
      int errcode = errno;
      printf("leds_service: ERROR: Failed to open %s: %d\n",
             CONFIG_ARDUSIMPLE_LEDS_DEVPATH, errcode);
      goto errout_with_fd_pwm;
    }

  /* Get the set of LEDs supported */

  ret = ioctl(fd_led, ULEDIOC_SUPPORTED,
              (unsigned long)((uintptr_t)&supported));
  if (ret < 0)
    {
      int errcode = errno;
      printf("leds_service: ERROR: ioctl(ULEDIOC_SUPPORTED) failed: %d\n",
             errcode);
      goto errout_with_fd_led;
    }

  /* Subscribe the uORB topics */

  memset(&battery_stat, 0, sizeof(struct battery_status_s));
  memset(&network_stat, 0, sizeof(struct network_status_s));
  memset(&btstack_stat, 0, sizeof(struct btstack_status_s));
  sub_battery  = orb_subscribe(ORB_ID(battery_status));
  sub_network  = orb_subscribe(ORB_ID(network_status));
  sub_btstack  = orb_subscribe(ORB_ID(btstack_status));
  uorb_started = false;

  /* Excluded any LEDs that not supported AND not in the set of LEDs the
   * user asked us to use.
   */

  printf("leds_service: Supported LEDs 0x%02x\n", (unsigned int)supported);
  supported &= CONFIG_ARDUSIMPLE_LEDS_LEDSET;

  /* Start the LED animation */

  led_animation(fd_pwm, true, &led_info);
  led_info.frequency = 200;

  uint8_t  effect_mask = LED_MASK_RED;
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

          /* BT stack status LED */

          if (btstack_stat.state)
            {
              effect_mask |= LED_MASK_BLUE;
            }
          else
            {
              effect_mask &= ~LED_MASK_BLUE;
            }

          /* WiFi status LED */

          if (network_stat.wlan0.link || network_stat.wlan1.link)
            {
              effect_mask |= LED_MASK_GREEN;
            }
          else
            {
              effect_mask &= ~LED_MASK_GREEN;
            }

          /* Update the LED effects */

          led_effect(fd_pwm,
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

          /* Battery status logic */

          if (uorb_started)
            {
              ledset = led_logic(battery_stat.percent,
                                 battery_stat.cable,
                                 battery_stat.state);
            }
          else
            {
              ledset = 0x06;
            }

          /* Update the LED(s) */

          ret = ioctl(fd_led, ULEDIOC_SETALL, ledset);
          if (ret < 0)
            {
              int errcode = errno;
              printf("led_daemon: ERROR: ioctl(ULEDIOC_SUPPORTED) failed: %d\n",
                    errcode);
              goto errout_with_fd_led;
            }
        }

      /* Check the battery status */

      if ((orb_check(sub_battery, &updated) == OK) && updated)
        {
          /* Get the published data */

          if (orb_copy(ORB_ID(battery_status), sub_battery, &battery_stat) == OK)
            {
              /* Set the flag to inform data has been collected */

              uorb_started = true;

              /* Send the collected infos to the console */

              printf("State: %d, Health: %d, Voltage: %d, Percentage: %d, Cable: %d\n",
                      battery_stat.state,
                      battery_stat.health,
                      battery_stat.voltage,
                      battery_stat.percent,
                      battery_stat.cable);
            }
        }

      /* Check the network status topic */

      if ((orb_check(sub_network, &updated) == OK) && updated)
        {
          /* Get the published data */

          if (orb_copy(ORB_ID(network_status), sub_network, &network_stat) == OK)
            {
              /* Update the LED animation according to status */

            }
        }

      /* Check the bluetooth stack status topic */

      if ((orb_check(sub_btstack, &updated) == OK) && updated)
        {
          /* Get the published data */

          if (orb_copy(ORB_ID(btstack_status), sub_btstack, &btstack_stat) == OK)
            {
              /* Update the LED animation according to status */

            }
        }

      /* Wait for a 10ms */

      usleep(10 * 1000L);
    }

errout_with_fd_led:
  close(fd_led);

errout_with_fd_pwm:
  close(fd_pwm);

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
                    CONFIG_ARDUSIMPLE_LEDS_PRIORITY,
                    CONFIG_ARDUSIMPLE_LEDS_STACKSIZE,
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
