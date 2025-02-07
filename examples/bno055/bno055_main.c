/****************************************************************************
 * apps/examples/bno055/bno055_main.c
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
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/bno055.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

/****************************************************************************
 * Private data
 ****************************************************************************/

static bool g_sensor_daemon_started;

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

      g_sensor_daemon_started = false;
      printf("sensor_daemon: Terminated.\n");
    }
  else
    {
      printf("\nsigterm_action: Received signo=%d siginfo=%p arg=%p\n",
             signo, siginfo, arg);
    }
}

/****************************************************************************
 * Name: open_sensor
 ****************************************************************************/
static int open_sensor(void)
{
  int pwr_mode;
  int opr_mode;
  int fd, ret;

  /* Open lowerhalf file to be able to read the data */

  fd = open(CONFIG_EXAMPLES_BNO055_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "Failed to open smart sensor, errno=%d\n",
              errno);
      return -1;
    }

  /* Perform a reset */

  ret = ioctl(fd, SNIOC_RESET, 0);
  if (ret != 0)
    {
      fprintf(stderr, "Failed to reset smart sensor, errno=%d\n",
              errno);
      close(fd);
      return -1;
    }

  /* Set the power mode of sensor */

  pwr_mode = BNO055_PWMODE_NORMAL;
  ret = ioctl(fd, SNIOC_SETPOWERMODE, &pwr_mode);
  if (ret != 0)
    {
      fprintf(stderr, "Failed to set smart sensor power mode, errno=%d\n",
              errno);
      close(fd);
      return -1;
    }

  /* Set the operation mode of sensor */

  opr_mode = BNO055_OPMODE_FUSION;
  ret = ioctl(fd, SNIOC_SETOPERMODE, &opr_mode);
  if (ret != 0)
    {
      fprintf(stderr, "Failed to set smart sensor power mode, errno=%d\n",
              errno);
      close(fd);
      return -1;
    }

  return fd;
}

/****************************************************************************
 * Name: sensor_daemon
 ****************************************************************************/

static int sensor_daemon(int argc, char *argv[])
{
  struct bno055_reports_s sensor_report;
  struct sigaction act;
  pid_t mypid;
  int fd, ret;

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
  g_sensor_daemon_started = true;
  printf("\nsensor_daemon (pid# %d): Running\n", mypid);

  /* This example works when all of the sub-sensors of
   * the BNO055 are enabled.
   */

  /* Open and configure the sensor */

  fd = open_sensor();
  if (fd < 0)
    {
      goto errout;
    }

  /* Poll event list */

  struct pollfd pfds =
  {
      .fd = fd,
      .events = POLLIN
  };

  /*
   * The first measurements are not accurate. The longer the wait, the more
   * accurate the results are.
   */

  usleep(1000 * 1000L);

  while (g_sensor_daemon_started == true)
    {

      do
        {
          /* Poll the sensor */

          ret = poll(&pfds, 1, 100);
          if (ret < 0)
            {
              if (g_sensor_daemon_started)
                {
                  fprintf(stderr, "Failed to poll smart sensor, errno=%d\n",
                                  errno);
                  goto errout_with_fd;
                }
            }

        } while (ret == 0);

      if (pfds.revents & POLLIN)
        {
          /* Read sensor data */

          ret = read(pfds.fd, &sensor_report, sizeof(sensor_report));
          if (ret != sizeof(sensor_report))
            {
              if (g_sensor_daemon_started)
                {
                  fprintf(stderr, "Failed to read smart sensor, errno=%d\n",
                                  errno);
                  goto errout_with_fd;
                }
            }

          /* Check the sensor data */

          switch (sensor_report.opr_mode)
            {
              case BNO055_OPMODE_RAW:
                {
                  /* Show us the data */

                  if (g_sensor_daemon_started)
                    {
                      printf("Ax:%d Ay:%d Az:%d\n", sensor_report.raw.accel.x,
                                                    sensor_report.raw.accel.y,
                                                    sensor_report.raw.accel.z);
                    }
                }
                break;

              case BNO055_OPMODE_FUSION:
                {
                  /* Show us the data */

                  if (g_sensor_daemon_started)
                    {
                      printf("Roll:%f Pitch:%f Yaw:%f\n", (sensor_report.fusion.euler.r)/16.0f,
                                                          (sensor_report.fusion.euler.p)/16.0f,
                                                          (sensor_report.fusion.euler.h)/16.0f);
                    }
                }
                break;
            }

        }
    }

  /* treats signal termination of the task
   * task terminated by a SIGTERM
   */

  exit(EXIT_SUCCESS);

errout_with_fd:
  close(fd);

errout:
  g_sensor_daemon_started = false;
  printf("sensor_daemon: Terminating\n");

  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * bno055_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("bno055_main: Starting the sensor_daemon\n");
  if (g_sensor_daemon_started)
    {
      printf("bno055_main: sensor_daemon already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("sensor_daemon",
                    CONFIG_EXAMPLES_BNO055_PRIORITY,
                    CONFIG_EXAMPLES_BNO055_STACKSIZE,
                    sensor_daemon,
                    argv);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bno055_main: ERROR: Failed to start sensor_daemon: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("bno055_main: sensor_daemon started\n");
  return EXIT_SUCCESS;
}
