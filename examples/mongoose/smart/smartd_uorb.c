/****************************************************************************
 * apps/examples/mongoose/smart/smartd_uorb.c
 *
 *   Copyright (C) 2007, 2009-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Copyright (c) 2001, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Adam Dunkels.
 * 4. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>

#include <netutils/netlib.h>
#include <uORB/uORB.h>
#include <mongoose.h>
#include <btstack_status.h>

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROOT_FS       (&mg_fs_posix)
#define ROOT_DIR      "/data0/www"
#define HOST_ADR      "0.0.0.0"

#define NPOLLFDS      (1)
#define POLL_DELAY    (1)
#define FRAME_HDRLEN  (9)

#define SENSOR_RATE_SLOW  (1000 * 1000)   /* uS */
#define SENSOR_RATE_FAST  ( 100 * 1000)   /* uS */

/****************************************************************************
 * Private Type
 ****************************************************************************/

/* Service data type */

struct httpd_serv_s
{
  bool          initialized; /* Networking is initialized */
  volatile bool stop;        /* Request daemon to exit    */
  volatile bool running;     /* The daemon is running     */
  pid_t         pid;         /* Task ID of the daemon     */
};

/* Server data type */

struct httpd_data_s
{
  struct mg_http_serve_opts opts;
  struct mg_mgr mgr;
  struct orb_object uorb;
  struct pollfd *fds;
  struct
  {
    int fd;
    struct btstack_status_s st;
  } bt;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Service data */

static struct httpd_serv_s g_httpd_serv = {0};

/* Server data */

static struct httpd_data_s g_httpd_data;

/* JSON header */

static const char *s_json_header =
    "Content-Type: application/json\r\n"
    "Cache-Control: no-cache\r\n";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: send_to_conn
 ****************************************************************************/

static int send_to_conn(struct mg_connection *conn, FAR const void *buf, size_t len, int op)
{
  char  *msg_buf = (char *)buf;
  char   msg_hdr[FRAME_HDRLEN+1] = "nmea.XXX#";
  size_t msg_len = len + FRAME_HDRLEN;

  /* Prepare the formatted message */

  if (msg_len < SENSOR_GNSS_RAWDATA_SIZE)
    {
      /* Prepare the header */

      msg_hdr[5] = msg_buf[3];
      msg_hdr[6] = msg_buf[4];
      msg_hdr[7] = msg_buf[5];

      /* Shift the incoming message */

      memmove(&msg_buf[FRAME_HDRLEN], msg_buf, len);

      /* Copy the frame header */

      memcpy(msg_buf, msg_hdr, FRAME_HDRLEN);

      /* Send it to the client */

      return mg_ws_send(conn, msg_buf, msg_len, op);
    }

  return 0;
}

/****************************************************************************
 * Name: send_to_ws
 ****************************************************************************/

static size_t send_to_ws(struct mg_mgr *mgr, FAR const void *buf, size_t len, int op)
{
  /* Traverse over all connections */

  for (struct mg_connection *c = mgr->conns; c != NULL; c = c->next)
    {
      /* Send only to marked connections */

      if (c->data[0] == 'W')
        {
          /* Send the message to the client connection */

          send_to_conn(c, buf, len, op);
        }
    }

    return 0;
}

/****************************************************************************
 * Name: uorb_ondata
 *
 * Description:
 *   Print topic data by its print_message callback.
 *
 * Input Parameters:
 *   meta         The uORB metadata.
 *   fd           Subscriber handle.
 *
 * Returned Value:
 *   0 on success copy, otherwise -1
 ****************************************************************************/

static int uorb_ondata(struct httpd_data_s *priv, FAR const struct orb_metadata *meta, int fd)
{
  char buffer[meta->o_size];
  FAR struct sensor_gnss_raw *gnss = (FAR struct sensor_gnss_raw *)buffer;
  int ret;

  ret = orb_copy(meta, fd, buffer);
  if (ret == OK)
    {
      /* Send the received data to the websocket connection */

      send_to_ws(&priv->mgr, gnss->buf, gnss->len, WEBSOCKET_OP_TEXT);

      /* Skip the read data */

      orb_ioctl(fd, SNIOC_SKIP_BUFFER,
                (unsigned long)(uintptr_t)&meta->o_size);
    }

  return ret;
}

/****************************************************************************
 * Name: sensor_subscribe
 ****************************************************************************/

static struct pollfd *sensor_subscribe(struct httpd_data_s *priv,
                                       FAR const char *topic_name,
                                       int topic_interval,
                                       int topic_latency)
{
  size_t len;
  int fd;

  /* Allocate poll data */

  priv->fds = malloc(sizeof(struct pollfd));
  if (!priv->fds)
    {
      return NULL;
    }

  /* get the object meta */

  len = strlen(topic_name) - 1;
  priv->uorb.instance = topic_name[len] - '0';
  priv->uorb.meta = orb_get_meta(topic_name);

  /* subscribe to the topic */

  fd = orb_subscribe_multi(priv->uorb.meta, priv->uorb.instance);
  if (fd < 0)
    {
      free(priv->fds);
      return NULL;
    }

  /* set the interval */

  if (topic_interval != 0)
    {
      orb_set_interval(fd, topic_interval);

      if (topic_latency != 0)
        {
          orb_set_batch_interval(fd, topic_latency);
        }
    }

  priv->fds->fd = fd;
  priv->fds->events = POLLIN;

  return priv->fds;
}

/****************************************************************************
 * Name: sensor_poll
 ****************************************************************************/

static void sensor_poll(void *param)
{
  struct httpd_data_s *priv = (struct httpd_data_s *)param;
  struct btstack_status_s stat;
  bool updated;

  /* Check the private data validity */

  if (priv == NULL)
    {
      return;
    }

  /* Check the uORB topic event */

  if (priv->fds)
    {
      while (poll(priv->fds, NPOLLFDS, POLL_DELAY) == NPOLLFDS)
        {
          if (priv->fds->revents & POLLIN)
            {
              if (uorb_ondata(priv, priv->uorb.meta, priv->fds->fd) < 0)
                {
                  break;
                }
            }
        }
    }

  /* Check the bluetooth stack status topic */

  if ((orb_check(priv->bt.fd, &updated) == OK) && updated)
    {
      /* Get the published data */

      if (orb_copy(ORB_ID(btstack_status), priv->bt.fd, &stat) == OK)
        {
          /* Is there any change at the BT status ? */

          if (memcmp(&priv->bt.st, &stat, sizeof(stat)) != 0)
          {
            /* Update the BT status */

            memcpy(&priv->bt.st, &stat, sizeof(stat));

            /* Update sensor period according to BT status */

            if (priv->bt.st.state)
              {
                /* BT/BLE is connected. Set the sensor rate to slow */

                orb_set_interval(priv->fds->fd, SENSOR_RATE_SLOW);
              }
            else
              {
                /* BT/BLE is not connected. Set the sensor rate to fast */

                orb_set_interval(priv->fds->fd, SENSOR_RATE_FAST);
              }
            }
        }
    }
}

/****************************************************************************
 * Name: handle_battery
 ****************************************************************************/

static void handle_battery(struct mg_connection *c,
                           struct mg_http_message *hm)
{
  mg_http_reply(c, 200, s_json_header,
    "{\"level\":45,\"charging\":true,\"capacity\":90,\"cycles\":300}");
}

/****************************************************************************
 * Name: handle_mode
 ****************************************************************************/

static void handle_mode(struct mg_connection *c,
                        struct mg_http_message *hm)
{
  if (mg_match(hm->method, mg_str("PUT"), NULL)) {
    printf("Mode PUT: %s\r\n", (char *) hm->body.buf);
    mg_http_reply(c, 200, s_json_header, hm->body.buf);
  }
  else if (mg_match(hm->method, mg_str("GET"), NULL)) {
    printf("Mode GET: %s\r\n", (char *) hm->body.buf);
    mg_http_reply(c, 200, s_json_header, "{ \"mode\": \"rover\" }");
  }
}

/****************************************************************************
 * Name: ev_handler
 ****************************************************************************/

static void ev_handler(struct mg_connection *c, int ev, void *ev_data)
{
  struct httpd_data_s *priv = &g_httpd_data;

  /* Connection accepted  */

  if (ev == MG_EV_ACCEPT)
    {
      /* TLS listener! */

      if (c->fn_data != NULL)
        {
          struct mg_tls_opts opts = {0};
          opts.cert = mg_file_read(priv->opts.fs, "/data0/certs/server_cert.pem");
          opts.key = mg_file_read(priv->opts.fs,"/data0/certs/server_key.pem");
          mg_tls_init(c, &opts);
          free(opts.cert.buf);
          free(opts.key.buf);
        }
    }

  /* Connection closed */

  else if (ev == MG_EV_CLOSE)
    {
      /* Clear the data  */

      c->data[0] = 0;
		}

  /* Websocket handshake done */

	else if (ev == MG_EV_WS_OPEN)
    {
      /* When WS handhake is done, mark us as WS client */

      c->data[0] = 'W';
    }

  /* Websocket msg, text or bin */

	else if (ev == MG_EV_WS_MSG)
		{
	    /* Got websocket frame. Received data is wm->data. */

	}

  /* Full HTTP request/response */

  else if (ev == MG_EV_HTTP_MSG)
    {
      struct mg_http_message *hm = ev_data;

      if (mg_match(hm->uri, mg_str("/websocket"), NULL))
        {
            /* Upgrade to websocket. From now on, a connection is a full-duplex */
            /* Websocket connection, which will receive MG_EV_WS_MSG events. */

          mg_ws_upgrade(c, hm, NULL);
        }
      else if (mg_match(hm->uri, mg_str("/api/battery"), NULL))
        {
          handle_battery(c, hm);
        }
      else if (mg_match(hm->uri, mg_str("/api/mode"), NULL))
        {
          handle_mode(c, hm);
        }
      else if (mg_match(hm->uri, mg_str("/assets/*"), NULL) ||
              mg_match(hm->uri, mg_str("/flags/*"), NULL) ||
              mg_match(hm->uri, mg_str("/icons/*"), NULL) ||
              mg_match(hm->uri, mg_str("/locales/*"), NULL) ||
              mg_match(hm->uri, mg_str("*.svg"), NULL))
        {
          mg_http_serve_dir(c, ev_data, &priv->opts);
        }
      else
        {
          mg_http_serve_dir(c, ev_data, &priv->opts);
        }
    }
}

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
      g_httpd_serv.stop = true;
    }
  else
    {
      printf("\nsigterm_action: Received signo=%d siginfo=%p arg=%p\n",
             signo, siginfo, arg);
    }
}

/****************************************************************************
 * Name: smart_daemon
 ****************************************************************************/

static int smart_daemon(int argc, FAR char *argv[])
{
  struct httpd_data_s *priv = &g_httpd_data;
  struct sigaction act;
  bool gnss_enab = false;
  int port = 8080;
  int s_port = -1;
  char rootdir[32];
  char hosturl[64];
  char hostadr[20];
  int option, ret;

  /* Clear the private data */

  memset(priv, 0, sizeof(struct httpd_data_s));

  /* Default values */

  strncpy(rootdir, ROOT_DIR, sizeof(rootdir));
  strncpy(hostadr, HOST_ADR, sizeof(hostadr));

  /* Register SIGTERM signal handler */

  memset(&act, 0, sizeof(struct sigaction));
  act.sa_sigaction = sigterm_action;
  act.sa_flags = SA_SIGINFO;

  /* Make sure that the terminate signal is unmasked */

  sigemptyset(&act.sa_mask);
  sigaddset(&act.sa_mask, SIGTERM);

  /* Set up so that sigterm_action will respond to SIGTERM */

  ret = sigaction(SIGTERM, &act, NULL);
  if (ret != 0)
    {
      fprintf(stderr, "Failed to install SIGTERM handler, errno=%d\n",
              errno);
      return (EXIT_FAILURE + 1);
    }

  /* The service has been started */

  g_httpd_serv.running = true;
  printf("Smart service [%d] started\n", g_httpd_serv.pid);

  /* Parse Argument */

  while ((option = getopt(argc, argv, "p:s:r:h:g")) != EOF)
    {
      switch (option)
      {
        case 'p':
          port = strtol(optarg, NULL, 0);
          if (port < 0)
            {
              goto errout;
            }
          break;

        case 's':
          s_port = strtol(optarg, NULL, 0);
          if (s_port < 0)
            {
              goto errout;
            }
          break;

        case 'r':
          strncpy(rootdir, optarg, sizeof(rootdir));
          break;

        case 'h':
          strncpy(hostadr, optarg, sizeof(hostadr));
          break;

        case 'g':
          gnss_enab = true;
          break;

        default:
          goto errout;
        }
    }

  if (gnss_enab)
    {
      /* Register the event topic(s) */

      sensor_subscribe(priv, "sensor_gnss_raw0", SENSOR_RATE_SLOW, 0);

      /* Sunscribe the BT/BLE status to change sensor period */

      priv->bt.fd = orb_subscribe(ORB_ID(btstack_status));
      memset(&priv->bt.st, 0xff, sizeof(priv->bt.st));
    }

  /* Initialize the options */

  priv->opts.fs = ROOT_FS;
  priv->opts.root_dir = rootdir;

  /* Set log level */

  mg_log_set(MG_LL_NONE);

  /* Initialise event manager */

  mg_mgr_init(&priv->mgr);

  /* Create HTTP listener */

  snprintf(hosturl, sizeof(hosturl), "http://%s:%d", hostadr, port);
  mg_http_listen(&priv->mgr, hosturl, ev_handler, NULL);

  /* Create HTTPS listener */

  if (s_port != -1)
    {
      snprintf(hosturl, sizeof(hosturl), "https://%s:%d", hostadr, s_port);
      mg_http_listen(&priv->mgr, hosturl, ev_handler, (void *) 1);
    }

  /* Add the sensor poll timer */

  mg_timer_add(&priv->mgr, 100,
               MG_TIMER_RUN_NOW | MG_TIMER_REPEAT,
               sensor_poll, priv);

  /* Infinite event loop */

  while (g_httpd_serv.stop == 0)
    {
      /* Event manager poll */

      mg_mgr_poll(&priv->mgr, 100);
    }

errout:

  /* Unregister the event topic(s) */

  if (priv->fds != NULL)
    {
      orb_unsubscribe(priv->fds->fd);
      free(priv->fds);
    }

  /* free the allocated resources */

  mg_mgr_free(&priv->mgr);

  /* Reset the service data */

  g_httpd_serv.running = false;
  g_httpd_serv.stop    = false;
  g_httpd_serv.pid     = -1;

#ifndef CONFIG_NSH_NETINIT
  /* We are running standalone (as opposed to a NSH built-in app). Therefore
   * we should not exit after httpd failure.
   */

  while (1)
    {
      sleep(3);
      printf("mongoose_main: Still running\n");
      fflush(stdout);
    }

#else /* CONFIG_NSH_NETINIT */
  /* We are running as a NSH built-in app.  Therefore we should exit.  This
   * allows to 'kill -9' the mongoose app, assuming it was started as a
   * background process.  For example:
   *
   *    nsh> mongoose &
   *    mongoose [6:100]
   *    nsh> Starting mongoose
   *
   *    nsh> kill -9 6
   *    nsh> mongoose_main: Exiting
   */

  printf("mongoose_main: Exiting\n");

#endif /* CONFIG_NSH_NETINIT */

  fflush(stdout);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smartd_start
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  /* Check if we have already initialized the service */

  if (!g_httpd_serv.initialized)
    {
      /* Initialize service state */

      g_httpd_serv.initialized = true;
      g_httpd_serv.pid         = -1;
      g_httpd_serv.stop        = false;
      g_httpd_serv.running     = false;
    }

  /* Then start the new service (if it is not already running) */

  if (g_httpd_serv.stop && g_httpd_serv.running)
    {
      printf("Waiting for Smart service [%d] to stop\n", g_httpd_serv.pid);
      return EXIT_FAILURE;
    }

  if (!g_httpd_serv.running)
    {
      printf("Starting the Smart service\n");
      g_httpd_serv.pid = task_create("smart_service",
                                    CONFIG_EXAMPLES_MONGOOSE_PRIORITY,
                                    CONFIG_EXAMPLES_MONGOOSE_STACKSIZE,
                                    smart_daemon, argv + 1);
      if (g_httpd_serv.pid < 0)
        {
          printf("Failed to start the Smart service: %d\n", errno);
          return EXIT_FAILURE;
        }
    }
  else
    {
      printf("Smart service [%d] is running\n", g_httpd_serv.pid);
    }

  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: smartd_stop
 ****************************************************************************/

int smartd_stop(int argc, FAR char *argv[])
{
  if (!g_httpd_serv.initialized || !g_httpd_serv.running)
    {
      printf("The Smart service not running\n");
      return EXIT_FAILURE;
    }

  printf("Stopping the Smart service, pid=%d\n", g_httpd_serv.pid);
  g_httpd_serv.stop = true;

  return EXIT_SUCCESS;
}