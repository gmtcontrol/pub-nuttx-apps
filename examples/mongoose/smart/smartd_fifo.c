/****************************************************************************
 * apps/examples/mongoose/smart/smartd_fifo.c
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
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>

#include <netutils/netlib.h>
#include <mongoose.h>

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROOT_FS       (&mg_fs_posix)
#define ROOT_DIR      "/data0/www"
#define HOST_ADR      "0.0.0.0"
#define FIFO_PATH     "/var/gnss0"

#define NPOLLFDS      (19)
#define GPSFIFODX     (0)
#define POLL_DELAY    (1000)  /* 1 seconds */
#define NMEA_MAXLEN   (116)
#define FRAME_HDRLEN  (9)

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

  if (msg_len < NMEA_MAXLEN)
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
 * Name: gpoll_thread
 ****************************************************************************/

static void *gpoll_thread(pthread_addr_t pvarg)
{
  struct httpd_data_s *priv = (struct httpd_data_s *)pvarg;
  static char buffer[CONFIG_DEV_FIFO_SIZE];
  struct pollfd fds[NPOLLFDS];
  char line[NMEA_MAXLEN];
  int i, fd, cnt, ret;
  ssize_t nbytes;
  bool timeout;
  bool pollin;
  int nevents;
  char ch;

  /* Open the FIFO for non-blocking read */

  fd = open(FIFO_PATH, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      printf("gpoll_thread: ERROR Failed to open FIFO %s: %d\n",
             FIFO_PATH, errno);
      close(fd);
      return (FAR void *)-1;
    }

  /* Loop forever */

  while (g_httpd_serv.stop == 0)
    {
      memset(fds, 0, sizeof(struct pollfd)*NPOLLFDS);
      fds[GPSFIFODX].fd      = fd;
      fds[GPSFIFODX].events  = POLLIN;
      fds[GPSFIFODX].revents = 0;

      timeout = false;
      pollin  = false;

      /* poll the FIFO */

      ret = poll(fds, NPOLLFDS, POLL_DELAY);

      if (ret < 0)
        {
          printf("gpoll_thread: ERROR poll failed: %d\n", errno);
        }
      else if (ret == 0)
        {
          printf("gpoll_thread: Timeout\n");
          timeout = true;
        }
      else if (ret > NPOLLFDS)
        {
          printf("gpoll_thread: ERROR poll reported: %d\n", errno);
        }
      else
        {
          pollin = true;
        }

      nevents = 0;
      for (i = 0; i < NPOLLFDS; i++)
        {
          if (timeout)
            {
              if (fds[i].revents != 0)
                {
                  printf("gpoll_thread: ERROR expected revents=00, "
                         "received revents[%d]=%08" PRIx32 "\n",
                         i, fds[i].revents);
                }
            }
          else if (pollin)
            {
              if (fds[i].revents == POLLIN)
                {
                  nevents++;
                }
              else if (fds[i].revents != 0)
                {
                  /*
                  printf("gpoll_thread: ERROR unexpected revents[%d]="
                         "%08" PRIx32 "\n", i, fds[i].revents);
                  */
                }
            }
        }

      if (pollin && nevents != ret)
        {
          /*
          printf("gpoll_thread: ERROR found %d events, "
                  "poll reported %d\n", nevents, ret);
          */
        }

      /* In any event, read until the pipe/serial  is empty */

      for (i = 0; i < NPOLLFDS; i++)
        {
          do
            {
              /* The pipe works differently, it returns whatever data
                * it has available without blocking.
                */

              nbytes = read(fds[i].fd, buffer, sizeof(buffer)-1);

              if (nbytes <= 0)
                {
                  if (nbytes == 0 || errno == EAGAIN)
                    {
                      if ((fds[i].revents & POLLIN) != 0)
                        {
                          printf("gpoll_thread: ERROR no read"
                                 " data[%d]\n", i);
                        }
                    }
                  else if (errno != EINTR)
                    {
                      printf("gpoll_thread: read[%d] failed: %d\n",
                             i, errno);
                    }

                  nbytes = 0;
                }
              else
                {
                  if (timeout)
                    {
                      printf("gpoll_thread: ERROR? Poll timeout, "
                              "but data read[%d]\n", i);
                      printf("               (might just be "
                             "a race condition)\n");
                    }

                  /* Read until we complete a line */
                  cnt = 0;
                  for (int j = 0; j < nbytes; j++)
                    {
                      ch = buffer[j];
                      if (ch != '\r' && ch != '\n' && cnt < NMEA_MAXLEN)
                        {
                          line[cnt++] = ch;
                        }

                      if ((ch == '\n') && (cnt > 2))
                        {
                          line[cnt] = '\0';
                          send_to_ws(&priv->mgr, line, cnt, WEBSOCKET_OP_TEXT);
                          cnt = 0;
                        }
                    }
                }

              /* Suppress error report if no read data on the next
               * time through
               */

              fds[i].revents = 0;
            }
          while (nbytes > 0);
        }
    }

  /* Won't get here */

  close(fd);
  return NULL;
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
#if defined(CONFIG_NSH_NETINIT) && defined(CONFIG_NET_TCP)
  struct httpd_data_s *priv = &g_httpd_data;
  struct sigaction act;
  bool gnss_enab = false;
  int exitcode = 0;
  int port = 8080;
  int s_port = -1;
  char rootdir[32];
  char hosturl[64];
  char hostadr[20];
  int option, ret;
  struct sched_param sparam;
  pthread_attr_t attr;
  pthread_t tid;

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

  /* The daemon has been started */

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

  /* Open FIFOs */

  if (gnss_enab)
    {
      if (mkfifo(FIFO_PATH, 0666) < 0)
        {
          if (EEXIST != errno)
            {
              printf("mongoose_main: mkfifo failed: %d\n", errno);
              exitcode = 1;
              goto errout;
            }
        }
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

  /* Start the GNSS message listeners */

  if (gnss_enab)
    {
      /* Thread attributes */

      pthread_attr_init(&attr);
#ifdef CONFIG_NETINIT_THREAD
      sparam.sched_priority = CONFIG_NETINIT_THREAD_PRIORITY - 1;
#else
      sparam.sched_priority = 100;
#endif
      pthread_attr_setschedparam(&attr, &sparam);
      pthread_attr_setstacksize(&attr, 2048);

      /* Create the thread */

      ret = pthread_create(&tid, &attr, gpoll_thread, &g_httpd_data);
      if (ret != 0)
        {
          printf("mongoose_main: Failed to create listener thread: %d\n", ret);
          exitcode = 2;
          goto errout;
        }
    }

  /* Infinite event loop */

  while (g_httpd_serv.stop == 0)
    {
      /* Event manager poll */

      mg_mgr_poll(&priv->mgr, 100);
    }

errout:

  /* free the allocated resources */

  mg_mgr_free(&priv->mgr);

  /* Reset the service data */

  g_httpd_serv.running = false;
  g_httpd_serv.stop    = false;
  g_httpd_serv.pid     = -1;
#endif /* CONFIG_NSH_NETINIT && CONFIG_NET_TCP */

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
  fflush(stdout);
  return exitcode;
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
