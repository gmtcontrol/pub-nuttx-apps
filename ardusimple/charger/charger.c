/****************************************************************************
 * apps/ardusimple/charger/charger_main.c
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
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <dsp.h>

#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <battery_status.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_CHARGER_DEVNAME
#  define CHARGER_DEVPATH CONFIG_EXAMPLES_CHARGER_DEVNAME
#else
#  define CHARGER_DEVPATH "/dev/batt0"
#endif

#ifdef CONFIG_EXAMPLES_ADC_DEVNAME
#  define ADC_DEVPATH CONFIG_EXAMPLES_ADC_DEVNAME
#else
#  define ADC_DEVPATH "/dev/adc0"
#endif

#ifndef CONFIG_ADC_GROUPSIZE
#  define CONFIG_ADC_GROUPSIZE 2
#endif

#ifdef CONFIG_EXAMPLES_VBUS_DEVNAME
#  define VBUS_DEVPATH CONFIG_EXAMPLES_VBUS_DEVNAME
#else
#  define VBUS_DEVPATH "/dev/gpio1"
#endif

#ifdef CONFIG_EXAMPLES_CHEN_DEVNAME
#  define CHEN_DEVPATH CONFIG_EXAMPLES_CHEN_DEVNAME
#else
#  define CHEN_DEVPATH "/dev/gpio5"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct battery_values_s
{
  float raw_vbat;
  float raw_vrev;
  float cal_vbat;
  float cal_vrev;
  float scale;
  float percent;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_charger_service_started;

static struct battery_status_s batt_stat;
static struct battery_values_s batt_vals;

/*
  ADC resolution : 12-bit (2^12 = 4096)
  ADC reference  : 3300mV
*/
#define BATMIN      (3000.0f)       // mV
#define BATMAX      (4120.0f)       // mV
#define REFVOL      (3420.0f)       // mV

#define CALIB_FACT  (REFVOL/4096)   // mV per bit

/* There voltage divider */
#define SCALE_R1    (4350.0f)       // ohm
#define SCALE_R2    (10000.0f)      // ohm

#define SCALE_FACT  ((SCALE_R1+SCALE_R2)/SCALE_R2)  // mV

/* Helper macros */
#define CALIB(r)    ((r) * (CALIB_FACT))
#define SCALE(c)    ((c) * (SCALE_FACT))

/* USB connection status */
const char *usb_status[2]=
{
  "Disconnected",
  "Connected"
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * show_bat_status
 ****************************************************************************/

static int show_bat_status(int fd, struct battery_status_s *stat)
{
  int status;
  int health;

  const char *statestr[] =
    {
      "UNKNOWN",
      "FAULT",
      "IDLE",
      "FULL",
      "CHARGING",
      "DISCHARGING"
    };

  const char *healthstr[] =
    {
      "UNKNOWN",
      "GOOD",
      "DEAD",
      "OVERHEAT",
      "OVERVOLTAGE",
      "UNSPEC_FAIL",
      "COLD",
      "WD_TMR_EXP",
      "SAFE_TMR_EXP",
      "DISCONNECTED"
    };

  int ret;

  ret = ioctl(fd, BATIOC_STATE, (unsigned long)(uintptr_t)&status);
  if (ret < 0)
    {
      printf("ioctl BATIOC_STATE failed. %d\n", errno);
      return -1;
    }

  ret = ioctl(fd, BATIOC_HEALTH, (unsigned long)(uintptr_t)&health);
  if (ret < 0)
    {
      printf("ioctl BATIOC_HEALTH failed. %d\n", errno);
      return -1;
    }

  printf("State: %s, Health: %s\n",
         statestr[status], healthstr[health]);

  /* Update the tpic data */

  if (stat)
    {
      stat->state  = status;
      stat->health = health;
    }

  return 0;
}

/****************************************************************************
 * config_charger
 ****************************************************************************/

int open_charger(void)
{
  FAR struct batio_operate_msg_s op;
  int errval;
  int current;
  int voltage;
  int fd, ret;

  /* open the battery charger device */

  fd = open(CHARGER_DEVPATH, O_RDWR);
  if (fd < 0)
    {
      errval = errno;
      printf("Charger device open error. (%d)\n", errval);
      return errval;
    }

  /* Set the input current limit (mA) */

  current = 1000;
  ret = ioctl(fd, BATIOC_INPUT_CURRENT, (unsigned long)(uintptr_t)&current);
  if (ret < 0)
    {
      errval = errno;
      printf("ioctl BATIOC_INPUT_CURRENT failed. (%d)\n", errval);
      close(fd);
      return errval;
    }

  /* Set the charge current (mA) */

  current = 512;
  ret = ioctl(fd, BATIOC_CURRENT, (unsigned long)(uintptr_t)&current);
  if (ret < 0)
    {
      errval = errno;
      printf("ioctl BATIOC_CURRENT failed. (%d)\n", errval);
      close(fd);
      return errval;
    }

  /* Set the charge voltage (mV) */

  voltage = 4208;
  ret = ioctl(fd, BATIOC_VOLTAGE, (unsigned long)(uintptr_t)&voltage);
  if (ret < 0)
    {
      errval = errno;
      printf("ioctl BATIOC_VOLTAGE failed. (%d)\n", errval);
      close(fd);
      return errval;
    }

  /* Enable charger termination  */

  op.operate_type = BATIO_OPRTN_EN_TERM;
  op.u32 = 1;
  ret = ioctl(fd, BATIOC_OPERATE, (unsigned long)(uintptr_t)&op);
  if (ret < 0)
    {
      errval = errno;
      printf("ioctl BATIOC_OPERATE(%d) failed. (%d)\n", op.operate_type, errval);
      close(fd);
      return errval;
    }

  /* Set charging mode */

  op.operate_type = BATIO_OPRTN_CHARGE;
  op.u32 = 0;
  ret = ioctl(fd, BATIOC_OPERATE, (unsigned long)(uintptr_t)&op);
  if (ret < 0)
    {
      errval = errno;
      printf("ioctl BATIOC_OPERATE(%d) failed. (%d)\n", op.operate_type, errval);
      close(fd);
      return errval;
    }

  /* Set system on mode (BATFET enable) */

  op.operate_type = BATIO_OPRTN_SYSON;
  op.u32 = 0;
  ret = ioctl(fd, BATIOC_OPERATE, (unsigned long)(uintptr_t)&op);
  if (ret < 0)
    {
      errval = errno;
      printf("ioctl BATIOC_OPERATE(%d) failed. (%d)\n", op.operate_type, errval);
      close(fd);
      return errval;
    }

  return fd;
}

/****************************************************************************
 * Name: charger_service
 ****************************************************************************/

static int charger_service(int argc, FAR char *argv[])
{
  struct adc_msg_s sample[CONFIG_ADC_GROUPSIZE];
  struct avg_filter_data_s avg_data[2];
  bool vbus=0, vbus_prev=0;
  int opt, verbose = 1;
  int ret, errval = 0;
  bool chgen;

  while ((opt = getopt(argc, argv, "v")) != -1)
    {
      switch (opt)
        {
          case 'v':
            verbose = 1;
            break;

          default:
            printf("Usage: %s [-v]\n", argv[0]);
            return 1;
        }
    }


  /* open the charger device */

  int fd_batt = open_charger();
  if (fd_batt < 0)
    {
      errval = errno;
      printf("Charger device open error.\n");
      goto errout;
    }


  /* open the ADC device */

  int fd_meas = open(ADC_DEVPATH, O_RDWR);
  if (fd_meas < 0)
    {
      errval = errno;
      printf("ADC device open error.\n");
      goto errout_batt;
    }

  /* open the VBUS GPIO */

  int fd_vbus = open(VBUS_DEVPATH, O_RDONLY);
  if (fd_vbus < 0)
    {
      errval = errno;
      printf("GPIO(VBUS) device open error.\n");
      goto errout_meas;
    }

  /* open the charge enable GPIO */

  int fd_chen = open(CHEN_DEVPATH, O_RDWR);
  if (fd_chen < 0)
    {
      errval = errno;
      printf("GPIO(CHEN) device open error.\n");
      goto errout_vbus;
    }

/* Initialize the filter data */

avg_filter_data_init(&avg_data[0], 0.0f, 0.8f);
avg_filter_data_init(&avg_data[1], 0.0f, 0.5f);

/* Create the publish file */

memset(&batt_vals, 0, sizeof(batt_vals));
memset(&batt_stat, 0, sizeof(batt_stat));

int batt_pub = orb_advertise(ORB_ID(battery_status), 0);

/* Measure the battery voltage forever */

while (1)
  {
    /* Show the battery status */

    if (verbose)
      {
        show_bat_status(fd_batt, &batt_stat);
      }

    /* Issue the software trigger to start ADC conversion */

    ret = ioctl(fd_meas, ANIOC_TRIGGER, 0);
    if (ret < 0)
      {
          break;
      }

    /* Read up to CONFIG_ADC_GROUPSIZE samples */

    size_t readsize = CONFIG_ADC_GROUPSIZE * sizeof(struct adc_msg_s);
    ssize_t nbytes = read(fd_meas, sample, readsize);

    /* Handle unexpected return values */

    if (nbytes < 0)
      {
        errval = errno;
        if (errval != EINTR)
          {
            errval = 3;
            goto errout_chen;
          }
      }

    /* Print the sample data on successful return */

    else if (nbytes > 0)
      {
        int nsamples = nbytes / sizeof(struct adc_msg_s);
        if (nsamples * sizeof(struct adc_msg_s) == nbytes)
          {
            /* Update the topic  data */

            batt_vals.raw_vbat = avg_filter(&avg_data[0], sample[0].am_data);
            batt_vals.raw_vrev = avg_filter(&avg_data[1], sample[1].am_data);
            batt_vals.cal_vbat = CALIB(batt_vals.raw_vbat);
            batt_vals.cal_vrev = CALIB(batt_vals.raw_vrev);
            batt_vals.scale    = SCALE(batt_vals.cal_vbat);
            batt_vals.percent  = 100 * ((batt_vals.scale - BATMIN) / (BATMAX - BATMIN));
            if (batt_vals.percent < 0) {
              batt_vals.percent = 0;
            }
            else if (batt_vals.percent > 100) {
              batt_vals.percent = 100;
            }

            batt_stat.voltage = batt_vals.scale;
            batt_stat.percent = batt_vals.percent;

            printf("Batt Raw  : %04ld\r\n",      (int32_t)(batt_vals.raw_vbat));
            printf("Batt Calib: %04ld (mV)\r\n", (int32_t)(batt_vals.cal_vbat));
            printf("Batt Scale: %04ld (mV)\r\n", (int32_t)(batt_vals.scale));
            printf("Batt Level: %02ld (%%)\r\n", (int32_t)(batt_vals.percent));
            printf("HRev Calib: %04ld (mV)\r\n", (int32_t)(batt_vals.cal_vrev));
          }
      }

      /* Read the VBUS status */

      ret = ioctl(fd_vbus, GPIOC_READ, (unsigned long)((uintptr_t)&vbus));

      /* Handle unexpected return values */

      if (ret < 0)
        {
          errval = errno;
          goto errout_chen;
        }

      /* Print the VBUS value on successful return */

      else
        {
          /* Update the topic data */

          batt_stat.cable = vbus;

          /* Check the VBUS value */

          if (vbus != vbus_prev)
            {
              /* Update the previous value */

              vbus_prev = vbus;

              /* USB connected */

              if (vbus)
                {
                  /* Start the charger */

                  chgen = false;
                }
              else
                {
                  /* Stop the charger to measure battery voltage */

                  chgen = true;
                }

              /* Update the charge enable pin status */

              ioctl(fd_chen, GPIOC_WRITE, (unsigned long)chgen);
            }

          printf("Cable: %s\r\n", (char*)usb_status[vbus&1]);
        }

    printf("\n");

    /* Publish the data */

    if (batt_pub > 0)
      {
        orb_publish(ORB_ID(battery_status), batt_pub, &batt_stat);
      }

    /* Wait for a 1000ms */

    usleep(10 * 1000L);
  }

errout_chen:
  close(fd_chen);

errout_vbus:
  close(fd_vbus);

errout_meas:
  close(fd_meas);

errout_batt:
  close(fd_batt);

errout:
  return errval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * charger_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("charger_main: Starting the charger service\n");
  if (g_charger_service_started)
    {
      printf("charger_main: charger service already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("charger_service",
                    CONFIG_ARDUSIMPLE_CHARGER_PRIORITY,
                    CONFIG_ARDUSIMPLE_CHARGER_STACKSIZE,
                    charger_service,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("charger_main: ERROR: Failed to start charger service: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("charger_main: charger service started\n");
  return EXIT_SUCCESS;
}