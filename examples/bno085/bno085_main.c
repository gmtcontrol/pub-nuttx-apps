/****************************************************************************
 * apps/examples/bno085/bno085_main.c
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
#include <nuttx/sensors/bno085.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

#define ARRAY_LEN(a)  (sizeof(a)/sizeof(a[0]))
#define RAD2DEG(r)    ((r)/M_PI*180)

/****************************************************************************
 * Private data
 ****************************************************************************/

static bool g_sensor_daemon_started;

/* Structure used when polling the sub-sensors */

static struct bno085_reports_s sensorReports[] =
{
    // Accelerometer, 100 Hz
    {SH2_ACCELEROMETER, {.reportInterval_us = 10000}},

    // Gyroscope calibrated , 100 Hz
    {SH2_GYROSCOPE_CALIBRATED, {.reportInterval_us = 10000}},

    // Magnetic field calibrated , 100 Hz
    {SH2_MAGNETIC_FIELD_CALIBRATED, {.reportInterval_us = 10000}},

    // Geo Magnetic field calibrated , 100 Hz
    {SH2_GEOMAGNETIC_ROTATION_VECTOR, {.reportInterval_us = 10000}},

};

/****************************************************************************
 * Public Functions
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
int open_sensor(void)
{
  int fd, ret, stat;

  /* Open lowerhalf file to be able to read the data */

  fd = open(CONFIG_EXAMPLES_BNO085_DEVPATH, O_RDONLY | O_NONBLOCK);
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

  /* Wait till reset has been completed */
  do
  {
    ret = ioctl(fd, SNIOC_GETSTATUS, (unsigned long)&stat);
    if (ret != 0)
      {
        fprintf(stderr, "Failed to get status from smart sensor, errno=%d\n",
                errno);
        close(fd);
        return -1;
      }

  } while (!(stat & BNO085_STATUS_READY));

  /* Configure the sensor */

  for (int n = 0; n < ARRAY_LEN(sensorReports); n++)
  {
    ret = ioctl(fd, SNIOC_SETCONFIG, &sensorReports[n]);
    if (ret != 0)
      {
        fprintf(stderr, "Failed to config smart sensor (%d), errno=%d\n",
                sensorReports[n].sensorId, errno);
        close(fd);
        return -1;
      }
    usleep(100 * 1000L);
  }

  return fd;
}

/****************************************************************************
 * Name: check_sensor
 ****************************************************************************/

int check_sensor(int fd)
{
  int ret, stat;

  ret = ioctl(fd, SNIOC_GETSTATUS, (unsigned long)&stat);
  if (ret != 0)
    {
      fprintf(stderr, "Failed to get status from smart sensor, errno=%d\n",
              errno);
      return ret;
    }

  /* Check the fault status */

  if (stat & BNO085_STATUS_FAULT)
    {
      printf("sensor_daemon: Faulty catched and recovered!\n");

      /* Perform a reset */

      ret = ioctl(fd, SNIOC_RESET, 0);
      if (ret != 0)
        {
          fprintf(stderr, "Failed to reset smart sensor, errno=%d\n",
                  errno);
          return ret;
        }

      /* Wait till reset has been completed */
      do
      {
        ret = ioctl(fd, SNIOC_GETSTATUS, (unsigned long)&stat);
        if (ret != 0)
          {
            fprintf(stderr, "Failed to get status from smart sensor, errno=%d\n",
                    errno);
            return ret;
          }

      } while (!(stat & BNO085_STATUS_READY));

      /* Configure the sensor */

      for (int n = 0; n < ARRAY_LEN(sensorReports); n++)
      {
        ret = ioctl(fd, SNIOC_SETCONFIG, &sensorReports[n]);
        if (ret != 0)
          {
            fprintf(stderr, "Failed to config smart sensor (%d), errno=%d\n",
                    sensorReports[n].sensorId, errno);
            return ret;
          }
        usleep(100 * 1000L);
      }
    }

    return 0;
}

/****************************************************************************
 * Name: sensor_daemon
 ****************************************************************************/

static int sensor_daemon(int argc, char *argv[])
{
  sh2_SensorValue_t    sensor_data;
  sh2_MagneticField_t  mag_data;
  sh2_Accelerometer_t  acc_data;
  sh2_Gyroscope_t      gyr_data;
  sh2_RotationVector_t qua_data;
  float roll, pitch, yaw;
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
   * the BNO085 are enabled.
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

  usleep(2000 * 1000L);

  while (g_sensor_daemon_started == true)
    {

      do
        {
          /* Check the sensor health */

          ret = check_sensor(fd);
          if (ret < 0)
            {
              if (g_sensor_daemon_started)
                {
                  goto errout_with_fd;
                }
            }

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

          ret = read(pfds.fd, &sensor_data, sizeof(sensor_data));
          if (ret != sizeof(sensor_data))
            {
              if (g_sensor_daemon_started)
                {
                  fprintf(stderr, "Failed to read smart sensor, errno=%d\n",
                                  errno);
                  goto errout_with_fd;
                }
            }

            /* Check the sensor data */

            switch (sensor_data.sensorId)
              {
                case SH2_ACCELEROMETER:
                  acc_data.x = sensor_data.un.accelerometer.x;
                  acc_data.y = sensor_data.un.accelerometer.y;
                  acc_data.z = sensor_data.un.accelerometer.z;
                  UNUSED(acc_data);
                  break;

                case SH2_GYROSCOPE_CALIBRATED:
                  gyr_data.x = sensor_data.un.gyroscope.x;
                  gyr_data.y = sensor_data.un.gyroscope.y;
                  gyr_data.z = sensor_data.un.gyroscope.z;
                  UNUSED(gyr_data);
                  break;

                case SH2_MAGNETIC_FIELD_CALIBRATED:
                  mag_data.x = sensor_data.un.magneticField.x;
                  mag_data.y = sensor_data.un.magneticField.y;
                  mag_data.z = sensor_data.un.magneticField.z;
                  UNUSED(mag_data);
                  break;

                case SH2_GEOMAGNETIC_ROTATION_VECTOR:
                  qua_data.i = sensor_data.un.arvrStabilizedGRV.i;
                  qua_data.j = sensor_data.un.arvrStabilizedGRV.j;
                  qua_data.k = sensor_data.un.arvrStabilizedGRV.k;
                  qua_data.real = sensor_data.un.arvrStabilizedGRV.real;
                  q_to_ypr(qua_data.real, qua_data.i, qua_data.j, qua_data.k, &yaw, &pitch, &roll);
                  break;
              }

            /* Show us the data */

            if (g_sensor_daemon_started)
              {
                printf("Roll:%f Pitch:%f Yaw:%f\n", RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
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
 * bno085_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("bno085_main: Starting the sensor_daemon\n");
  if (g_sensor_daemon_started)
    {
      printf("bno085_main: sensor_daemon already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("sensor_daemon",
                    CONFIG_EXAMPLES_BNO085_PRIORITY,
                    CONFIG_EXAMPLES_BNO085_STACKSIZE,
                    sensor_daemon,
                    argv);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bno085_main: ERROR: Failed to start sensor_daemon: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("bno085_main: sensor_daemon started\n");
  return EXIT_SUCCESS;
}
