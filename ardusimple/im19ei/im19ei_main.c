/****************************************************************************
 * apps/ardusimple/im19ei/im19ei_main.c
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
#include <sys/stat.h>
#include <sys/select.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>

#include <uORB/uORB.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define POLL_PERIOD_MS    (10)
#define POLL_DELAY_SEC    (1)
#define NPOLLFDS          (1)

#define IM19EI_CONF_SIZE  (6)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

typedef struct im19ei_conf
{
  const char *str;
  const int   len;
  const int   tmo;
} im19ei_conf_s;

typedef struct im19ei_data
{
  int fd1, fd2;
  bool started;
  bool configured;

  /* uORB Subscribers */

  struct
  {
    /* GNSS Raw */

    struct
    {
      struct orb_object  obj;
      FAR struct pollfd *fds;
    } gnss;

  } sub;
} im19ei_data_s;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Application data */

static im19ei_data_s g_im19ei_data =
{
  .fd1 = -1,
  .fd2 = -1,
  .started = false,
  .sub.gnss.fds = NULL,
  .sub.gnss.obj.meta = NULL,
  .sub.gnss.obj.instance = 0,
};

/* IM19EI configuration words */

static im19ei_conf_s g_im19ei_conf[IM19EI_CONF_SIZE] =
{
  /* System reset */

  {
    .str = "AT+SYSTEM_RESET\r\n",
    .len = 17,
    .tmo = 1000
  },

  /* Query the Firmware version */

  {
    .str = "AT+VERSION\r\n",
    .len = 12,
    .tmo = 1000
  },

  /* Binary NAVI positioning output */

  {
    .str = "AT+NAVI_OUTPUT=UART1,OFF\r\n",
    .len = 26,
    .tmo = 1000
  },

  /* MEMS raw output */

  {
    .str = "AT+MEMS_OUTPUT=UART1,OFF\r\n",
    .len = 26,
    .tmo = 1000
  },

  /* GNSS raw output */

  {
    .str = "AT+GNSS_OUTPUT=UART1,OFF\r\n",
    .len = 26,
    .tmo = 1000
  },

  /* Ascii type NAVI positioning output */

  {
    .str = "AT+NASC_OUTPUT=UART1,ON\r\n",
    .len = 25,
    .tmo = 1000
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open_serial
 ****************************************************************************/

static int open_serial(FAR const char *devpath, uint32_t baudrate)
{
  struct termios tio;
  int fd, ret, baud;

  /* Open the serial port */

  fd = open(devpath, O_RDWR);
  if (fd < 0)
    {
      printf("ERROR: Failed to open device path (%s)!\n",
             devpath);
      return -1;
    }

  /* Fill the termios struct with the current values. */

  ret = tcgetattr(fd, &tio);
  if (ret < 0)
    {
      printf("ERROR: Failed to get attributes (%d)!\n",
             errno);
      close(fd);
      return -1;
    }

  /* Configure a baud rate.
   * NuttX doesn't support different baud rates for RX and TX.
   * So, both cfisetospeed() and cfisetispeed() are overwritten
   * by cfsetspeed.
   */
  switch (baudrate)
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
      printf("ERROR: Failed to set baud rate (%d)\n",
             errno);
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

  /* Disable the HW flow control */

  tio.c_cflag &= ~CCTS_OFLOW;    /* Output flow control */
  tio.c_cflag &= ~CRTS_IFLOW;    /* Input flow control */

  /* Change the attributes now. */

  ret = tcsetattr(fd, TCSANOW, &tio);
  if (ret < 0)
    {
      /* Print the error code in the loop because at this
       * moment the serial attributes already changed
       */

      printf("ERROR: Failed to change attributes (%d)\n",
             errno);
      close(fd);
      return -1;
    }

  close(fd);

  /* Now, we should reopen the terminal with the new
   * attributes to see if they took effect;
   */

  /* Reopen the serial port */

  return open(devpath, O_RDWR);
}

/****************************************************************************
 * Name: sensor_subscribe
 ****************************************************************************/

static struct pollfd *sensor_subscribe(struct im19ei_data *priv,
                                       const char *topic_name,
                                       float topic_rate,
                                       int topic_latency)
{
  struct orb_object *obj = &priv->sub.gnss.obj;
  FAR struct pollfd *fds;
  size_t len;
  int fd;

  /* Allocate poll data */

  fds = malloc(sizeof(struct pollfd));
  if (!fds)
    {
      return NULL;
    }

  /* calculate the inverval */

  float interval = topic_rate ? (1000000 / topic_rate) : 0;

  /* get the object meta */

  len = strlen(topic_name) - 1;
  obj->instance = topic_name[len] - '0';
  obj->meta = orb_get_meta(topic_name);

  /* subscribe to the topic */

  fd = orb_subscribe_multi(obj->meta, obj->instance);
  if (fd < 0)
    {
      free(fds);
      return NULL;
    }

  /* set the interval */

  if (interval != 0)
    {
      orb_set_interval(fd, (unsigned)interval);

      if (topic_latency != 0)
        {
          orb_set_batch_interval(fd, topic_latency);
        }
    }

  fds->fd = fd;
  fds->events = POLLIN;

  /* Update the subsciber data */

  priv->sub.gnss.fds = fds;

  return fds;
}

/****************************************************************************
 * Name: sensor_ondata
 ****************************************************************************/

static int sensor_ondata(struct im19ei_data *priv,
                         const struct orb_metadata *meta,
                         int fd)
{
  char buffer[meta->o_size];
  FAR struct sensor_gnss_raw *gnss = (FAR struct sensor_gnss_raw *)buffer;
  int ret;

  ret = orb_copy(meta, fd, buffer);
  if (priv && ret == OK)
    {
      /* Forward all recevied data to the IM19EI port 2 */

      if (write(priv->fd2, gnss->buf, gnss->len) != gnss->len)
        {
          ret = -errno;
        }

      /* Skip the read data */

      orb_ioctl(fd, SNIOC_SKIP_BUFFER,
                (unsigned long)(uintptr_t)&meta->o_size);
    }

  return ret;
}

/****************************************************************************
 * Name: sensor_poll
 ****************************************************************************/

static int sensor_poll(struct im19ei_data *priv)
{
  int ret = ERROR;

  /* Check the uORB topic event */

  if (priv && priv->sub.gnss.fds != NULL)
    {
      struct orb_object *obj = &priv->sub.gnss.obj;
      FAR struct pollfd *fds =  priv->sub.gnss.fds;

      while (poll(fds, NPOLLFDS, POLL_DELAY_SEC) == NPOLLFDS)
        {
          if (fds->revents & POLLIN)
            {
              ret = sensor_ondata(priv, obj->meta, fds->fd);
              if (ret < 0)
                {
                  break;
                }
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: im19ei_proc
 ****************************************************************************/

static int im19ei_proc(struct im19ei_data *priv)
{
  static uint8_t cfg_idx = 0;
  im19ei_conf_s *conf;
  char ch, line[128];
  int cnt, ret = OK;

  /* Check arguments */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->fd1 != -1);

  /* Send configuration word to the IM19EI port 1 */

  if (cfg_idx < IM19EI_CONF_SIZE)
    {
      struct timeval timeout;
      fd_set set;

      /* clear the set */

      FD_ZERO(&set);

      /* add our file descriptor to the set */

      FD_SET(priv->fd1, &set);

      /* Get the currnt configuratin word */

      conf = &g_im19ei_conf[cfg_idx];

      /* Prepare the timeout */

      timeout.tv_sec  = 0;
      timeout.tv_usec = conf->tmo * 1000L;

      /* Send the configuration word */

      if (write(priv->fd1, conf->str, conf->len) == conf->len)
        {
          /* Echo the transmitted data */

          printf("T: %s", conf->str);
          usleep(conf->tmo * 1000L);

          /* Read until we complete a line */

          cnt = 0;
          do
            {
              /* Wait for event */

              ret = select(priv->fd1 + 1, &set, NULL, NULL, &timeout);
              if (ret > 0)
                {
                  /* there was data to read */

                  read(priv->fd1, &ch, 1);
                  if (ch != '\r' && ch != '\n')
                    {
                      line[cnt++ % sizeof(line)] = ch;
                    }
                }
            }
          while (ch != '\r' && ch != '\n' && ret > 0);

          /* Print the response */

          if (cnt > 0)
            {
              line[cnt++] = '\r';
              line[cnt++] = '\n';
              line[cnt  ] = '\0';
              printf("R: %s", line);
            }
        }
      else
        {
          ret = ERROR;
        }

      /* increment the configuration index */

      cfg_idx++;
    }
  else
    {
      /* Configuration done */

      priv->configured = true;

      /* Read until we complete a line */

      cnt = 0;
      do
        {
          /* there was data to read */

          ret = read(priv->fd1, &ch, 1);
          if (ret == 1)
            {
              if (ch != '\r' && ch != '\n')
                {
                  line[cnt++] = ch;
                }
            }
        }
      while (ch != '\r' && ch != '\n' && ret > 0);

      /* Print the response */

      if (cnt > 0)
        {
          line[cnt++] = '\r';
          line[cnt++] = '\n';
          line[cnt  ] = '\0';
          printf("R: %s", line);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: im19ei_service
 ****************************************************************************/

static int im19ei_service(int argc, char *argv[])
{
  struct im19ei_data *priv = &g_im19ei_data;

  UNUSED(argc);
  UNUSED(argv);

  /* Indicate that we are running */

  priv->started = true;
  printf("IM19EI Service Running\n");

  /* Open the IM19EI port 1 */

  priv->fd1 = open_serial(CONFIG_ARDUSIMPLE_IM19EI_PORT1,
                          CONFIG_ARDUSIMPLE_IM19EI_BAUD1);
  if (priv->fd1 < 0)
    {
      printf("ERROR: Failed to open IM19EI port 1 (%s)\n",
              CONFIG_ARDUSIMPLE_IM19EI_PORT1);
      goto errout;
    }

  /* Open the IM19EI port 2 */

  priv->fd2 = open_serial(CONFIG_ARDUSIMPLE_IM19EI_PORT2,
                          CONFIG_ARDUSIMPLE_IM19EI_BAUD2);
  if (priv->fd2 < 0)
    {
      printf("ERROR: Failed to open IM19EI port 2 (%s)\n",
              CONFIG_ARDUSIMPLE_IM19EI_PORT2);
      goto errout_with_fd1;
    }

  /* Register the event topic(s) */

  if (!sensor_subscribe(priv, "sensor_gnss_raw0", 1000/POLL_PERIOD_MS, 0))
    {
      printf("ERROR: Failed to subscribe GNSS raw data\n");
      goto errout_with_fd2;
    }

  /* Now loop forever */

  while (priv->started)
    {
      /* Process the IM19EI data */

      im19ei_proc(priv);

      /* Poll the raw GNSS topic */

      if (priv->configured)
        {
          sensor_poll(priv);
        }
    }

  orb_close(priv->sub.gnss.fds->fd);
  priv->sub.gnss.fds = NULL;

errout_with_fd2:
  close(priv->fd2);
  priv->fd2 = -1;

errout_with_fd1:
  close(priv->fd1);
  priv->fd1 = -1;

errout:
  priv->started = false;

  printf("IM19EI Service Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * im19_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("Starting the attitude service\n");
  if (g_im19ei_data.started)
    {
      printf("Attitude service already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("im19ei_service",
                    CONFIG_ARDUSIMPLE_IM19EI_PRIORITY,
                    CONFIG_ARDUSIMPLE_IM19EI_STACKSIZE,
                    im19ei_service,
                    NULL);
  if (ret < 0)
    {
      printf("ERROR: Failed to start attitude service (%d)\n",
             errno);
      return EXIT_FAILURE;
    }

  printf("Attitude service started\n");
  return EXIT_SUCCESS;
}
