/****************************************************************************
 * apps/examples/gps/gps_main.c
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

#include <stdio.h>
#include <fcntl.h>
#include <wchar.h>
#include <syslog.h>
#include <unistd.h>
#include <termios.h>

#include <minmea/minmea.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MINMEA_MAX_LENGTH    256

static int open_serial(void)
{
  struct termios tio;
  int fd, ret, baud;

  /* Open the GPS serial port */

  fd = open(CONFIG_EXAMPLES_GPS_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("Failed to open device path!\n");
      return -1;
    }

  /* Fill the termios struct with the current values. */

  ret = tcgetattr(fd, &tio);
  if (ret < 0)
    {
      printf("Error getting attributes: %d\n", errno);
      close(fd);
      return -1;
    }

  /* Configure a baud rate.
   * NuttX doesn't support different baud rates for RX and TX.
   * So, both cfisetospeed() and cfisetispeed() are overwritten
   * by cfsetspeed.
   */
  switch (CONFIG_EXAMPLES_GPS_BAUDRATE)
    {
      case 921600:
        baud = B921600;
        break;

      case 460800:
        baud = B460800;
        break;

      case 230400:
        baud = B230400;
        break;

      case 115200:
        baud = B115200;
        break;

      case 57600:
        baud = B57600;
        break;

      case 38400:
        baud = B38400;
        break;

      case 19200:
        baud = B19200;
        break;

      case 9600:
        baud = B9600;
        break;

      default:
        baud = B38400;
    }

  ret = cfsetspeed(&tio, baud);
  if (ret < 0)
    {
      printf("Error setting baud rate: %d\n", errno);
      close(fd);
      return -1;
    }

  /* Configure 1 stop bits. */

  tio.c_cflag &= ~CSTOPB;

  /* Disable parity. */

  tio.c_cflag &= ~PARENB;

  /* Change the data size to 8 bits */

  tio.c_cflag &= ~CSIZE; /* Clean the bits */
  tio.c_cflag |= CS8;    /* 8 bits */

#ifdef CONFIG_EXAMPLES_TERMIOS_DIS_HW_FC

  /* Disable the HW flow control */

  tio.c_cflag &= ~CCTS_OFLOW;    /* Output flow control */
  tio.c_cflag &= ~CRTS_IFLOW;    /* Input flow control */

#endif

  /* Change the attributes now. */

  ret = tcsetattr(fd, TCSANOW, &tio);
  if (ret < 0)
    {
      /* Print the error code in the loop because at this
       * moment the serial attributes already changed
       */

      printf("Error changing attributes: %d\n", errno);
      close(fd);
      return -1;

    }

  close(fd);

  /* Now, we should reopen the terminal with the new
   * attributes to see if they took effect;
   */

  /* Reopen the GPS serial port */

  fd = open(CONFIG_EXAMPLES_GPS_DEVPATH, O_RDONLY);

  return fd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * gps_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int cnt;
  char ch;
  char line[MINMEA_MAX_LENGTH];

  /* Open the GPS serial port */

  fd = open_serial();
  if (fd < 0)
    {
      printf("Failed to open GPS serial port!\n");
      exit(1);
    }

  /* Run forever */

  for (; ; )
    {
      /* Read until we complete a line */

      cnt = 0;
      do
        {
          read(fd, &ch, 1);
          if (ch != '\r' && ch != '\n')
            {
              line[cnt++] = ch;
            }
        }
      while (ch != '\r' && ch != '\n');

      line[cnt] = '\0';

      switch (minmea_sentence_id(line, false))
        {
          case MINMEA_SENTENCE_RMC:
            {
              struct minmea_sentence_rmc frame;

              if (minmea_parse_rmc(&frame, line))
                {
                  printf("Fixed-point Latitude...........: %ld\n",
                         minmea_rescale(&frame.latitude, 1000));
                  printf("Fixed-point Longitude..........: %ld\n",
                         minmea_rescale(&frame.longitude, 1000));
                  printf("Fixed-point Speed..............: %ld\n",
                         minmea_rescale(&frame.speed, 1000));
                  printf("Floating point degree latitude.: %2.6f\n",
                         minmea_tocoord(&frame.latitude));
                  printf("Floating point degree longitude: %2.6f\n",
                         minmea_tocoord(&frame.longitude));
                  printf("Floating point speed...........: %2.6f\n",
                         minmea_tocoord(&frame.speed));
                }
              else
                {
                    printf("$xxRMC sentence is not parsed\n");
                }
            }
            break;

          case MINMEA_SENTENCE_GGA:
            {
              struct minmea_sentence_gga frame;

              if (minmea_parse_gga(&frame, line))
                {
                  printf("Fix quality....................: %d\n",
                         frame.fix_quality);
                  printf("Altitude.......................: %ld\n",
                         frame.altitude.value);
                  printf("Tracked satellites.............: %d\n",
                         frame.satellites_tracked);
                }
              else
                {
                  printf("$xxGGA sentence is not parsed\n");
                }
            }
            break;

          default:
            {
            }
            break;
        }
    }

  return 0;
}
