/****************************************************************************
 * apps/ardusimple/gserv/gserv.c
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

#include <sys/stat.h>

#include <stdio.h>
#include <ctype.h>
#include <fcntl.h>
#include <termios.h>

#include <minmea/minmea.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NMEA_MAX_LENGTH   116
#define BUFF_MAX_LENGTH   1024

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hex2int
 ****************************************************************************/

static int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

/****************************************************************************
 * Name: nmea_check
 ****************************************************************************/

static bool nmea_check(const char *sentence, bool strict)
{
  uint8_t checksum = 0x00;

  /* A valid sentence starts with "$". */
  if (*sentence++ != '$')
    {
      return false;
    }

  /* The optional checksum is an XOR of all bytes between "$" and "*". */
  while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
    {
      checksum ^= *sentence++;
    }

  /* If checksum is present... */
  if (*sentence == '*')
    {
      // Extract checksum.
      sentence++;
      int upper = hex2int(*sentence++);
      if (upper == -1)
          return false;
      int lower = hex2int(*sentence++);
      if (lower == -1)
          return false;
      int expected = upper << 4 | lower;

      // Check for checksum mismatch.
      if (checksum != expected)
          return false;
    }
  else if (strict)
    {
      /* Discard non-checksummed frames in strict mode. */
      return false;
    }

  // The only stuff allowed at this point is a newline.
  while (*sentence == '\r' || *sentence == '\n')
    {
      sentence++;
    }

  if (*sentence)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: open_serial
 ****************************************************************************/

static int open_serial(void)
{
  struct termios tio;
  int fd, ret, baud;

  /* Open the GNSS serial port */

  fd = open(CONFIG_ARDUSIMPLE_GSERV_DEVPATH, O_RDONLY);
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
  switch (CONFIG_ARDUSIMPLE_GSERV_BAUDRATE)
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

  /* Reopen the GNSS serial port */

  fd = open(CONFIG_ARDUSIMPLE_GSERV_DEVPATH, O_RDONLY);

  return fd;
}

/****************************************************************************
 * Name: gserv_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  static uint8_t buff[BUFF_MAX_LENGTH];
  char line[NMEA_MAX_LENGTH];
  int nbytes, cnt, idx, ret;
  int fd_f = -1;
  int fd_g = -1;
  int exitcode = 0;
  char ch;

  /* Open the GNSS serial port */

  printf("\ngserv_main: Opening GNSS serial port %s\n", CONFIG_ARDUSIMPLE_GSERV_DEVPATH);
  fd_g = open_serial();
  if (fd_g < 0)
    {
      printf("gserv_main: Open the GNSS serial port failed: %d\n", errno);
      exitcode = 1;
      goto errout;
    }

  /* Open FIFOs */

  printf("gserv_main: Creating FIFO %s\n", CONFIG_ARDUSIMPLE_GSERV_FIFO);
  ret = mkfifo(CONFIG_ARDUSIMPLE_GSERV_FIFO, 0666);
  if (ret < 0)
    {
      if (EEXIST != errno)
        {
          printf("gserv_main: mkfifo failed: %d\n", errno);
          exitcode = 2;
          goto errout;
        }
    }

  /* Open the FIFOs for nonblocking, write */

  fd_f = open(CONFIG_ARDUSIMPLE_GSERV_FIFO, O_WRONLY | O_NONBLOCK);
  if (fd_f < 0)
    {
      printf("gserv_main: Failed to open FIFO %s for writing, errno=%d\n",
             CONFIG_ARDUSIMPLE_GSERV_FIFO, errno);
      exitcode = 3;
      goto errout;
    }

  /* Loop forever */

  for (; ; )
    {
      /* Read GNSS data to the temporary buffer */

      nbytes = read(fd_g, buff, BUFF_MAX_LENGTH);
      idx = 0;

      /* Wait till all data have been processed */

      while (nbytes > 0)
        {
          /* Continue until we complete a line */

          cnt = 0;
          do
            {
              ch = buff[idx++];
              line[cnt++] = ch;
            }
          while ((ch != '\n') && (cnt < NMEA_MAX_LENGTH));
          line[cnt] = '\0';

          /* Check the frame */

          if (nmea_check(line, true))
            {

              /* Write a message to the FIFO ...
               * this should wake the listener from the poll.
               */
              ret = write(fd_f, line, cnt);
              if (ret != cnt)
                {
                  printf("gserv_main: Write to FIFO failed: %d\n", errno);
                  exitcode = 4;
                  goto errout;
                }
            }
          else
            {
              /* Wrong frame format */

              line[cnt] = 0;
            }

          /* Update the processed bytes */

          nbytes -= cnt;
        }
    }

errout:
  if (fd_f >= 0)
    {
      close(fd_f);
    }

  if (fd_g >= 0)
    {
      close(fd_g);
    }

  fflush(stdout);
  return exitcode;
}
