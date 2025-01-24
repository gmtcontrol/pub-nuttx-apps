/****************************************************************************
 * apps/examples/mongoose/smart/httpd.c
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
#include <mongoose.h>


/****************************************************************************
 * External Functions
 ****************************************************************************/

extern bool     mg_ota_commit   (void);
extern bool     mg_ota_rollback (void);
extern int      mg_ota_status   (int fw);
extern uint32_t mg_ota_crc32    (int fw);
extern size_t   mg_ota_size     (int fw);
extern uint32_t mg_ota_timestamp(int fw);
extern void     mg_device_reset (void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

enum
{
  MG_OTA_UNAVAILABLE = 0,  // No OTA information is present
  MG_OTA_FIRST_BOOT = 1,   // Device booting the first time after the OTA
  MG_OTA_UNCOMMITTED = 2,  // Ditto, but marking us for the rollback
  MG_OTA_COMMITTED = 3     // The firmware is good
};

enum
{
  MG_FIRMWARE_CURRENT = 0,
  MG_FIRMWARE_PREVIOUS = 1
};

#define ROOT_FS   (&mg_fs_posix)
#define ROOT_DIR  "/data0/www"
#define HOST_ADR  "0.0.0.0"

/****************************************************************************
 * Private Type
 ****************************************************************************/

// Authenticated user.
// A user can be authenticated by:
//   - a name:pass pair, passed in a header Authorization: Basic .....
//   - an access_token, passed in a header Cookie: access_token=....
// When a user is shown a login screen, she enters a user:pass. If successful,
// a server responds with a http-only access_token cookie set.
struct user_s
{
  const char *name;
  const char *pass;
  const char *access_token;
};

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
 * Name: authenticate
 ****************************************************************************/

// Parse HTTP requests, return authenticated user or NULL
static struct user_s *authenticate(struct mg_http_message *hm)
{
  // In production, make passwords strong and tokens randomly generated
  // In this example, user list is kept in RAM. In production, it can
  // be backed by file, database, or some other method.
  static struct user_s users[] =
  {
      {"admin", "admin", "admin_token"},
      {"user1", "user1", "user1_token"},
      {"user2", "user2", "user2_token"},
      {NULL, NULL, NULL},
  };
  char user[64], pass[64];
  struct user_s *u, *result = NULL;

  mg_http_creds(hm, user, sizeof(user), pass, sizeof(pass));
  MG_VERBOSE(("user [%s] pass [%s]", user, pass));

  if (user[0] != '\0' && pass[0] != '\0')
  {
    /* Both user and password is set, search by user/password */

    for (u = users; result == NULL && u->name != NULL; u++) {
      if (strcmp(user, u->name) == 0 && strcmp(pass, u->pass) == 0) {
        result = u;
      }
    }
  } else if (user[0] == '\0') {
    /* Only password is set, search by token */

    for (u = users; result == NULL && u->name != NULL; u++) {
      if (strcmp(pass, u->access_token) == 0) {
        result = u;
      }
    }
  }

  return result;
}

/****************************************************************************
 * Name: handle_login
 ****************************************************************************/

static void handle_login(struct mg_connection *c, struct user_s *u)
{
  char cookie[256];

  mg_snprintf(cookie, sizeof(cookie),
              "Set-Cookie: access_token=%s; Path=/; "
              "%sHttpOnly; SameSite=Lax; Max-Age=%d\r\n",
              u->access_token, c->is_tls ? "Secure; " : "", 3600 * 24);

  mg_http_reply(c, 200, cookie, "{%m:%m}", MG_ESC("user"), MG_ESC(u->name));
}

/****************************************************************************
 * Name: handle_logout
 ****************************************************************************/

static void handle_logout(struct mg_connection *c)
{
  char cookie[256];

  mg_snprintf(cookie, sizeof(cookie),
              "Set-Cookie: access_token=; Path=/; "
              "Expires=Thu, 01 Jan 1970 00:00:00 UTC; "
              "%sHttpOnly; Max-Age=0; \r\n",
              c->is_tls ? "Secure; " : "");

  mg_http_reply(c, 200, cookie, "true\n");
}

/****************************************************************************
 * Name: handle_debug
 ****************************************************************************/

static void handle_debug(struct mg_connection *c, struct mg_http_message *hm)
{
  int level = (int) mg_json_get_long(hm->body, "$.level", MG_LL_DEBUG);
  mg_log_set(level);
  mg_http_reply(c, 200, "", "Debug level set to %d\n", level);
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
 * Name: handle_firmware_upload
 ****************************************************************************/

static void handle_firmware_upload(struct mg_connection *c,
                                   struct mg_http_message *hm)
{
  char name[64], offset[20], total[20];
  struct mg_str data = hm->body;
  long ofs = -1, tot = -1;

  name[0] = offset[0] = '\0';
  mg_http_get_var(&hm->query, "name", name, sizeof(name));
  mg_http_get_var(&hm->query, "offset", offset, sizeof(offset));
  mg_http_get_var(&hm->query, "total", total, sizeof(total));
  MG_INFO(("File %s, offset %s, len %lu", name, offset, data.len));

  if ((ofs = mg_json_get_long(mg_str(offset), "$", -1)) < 0 ||
      (tot = mg_json_get_long(mg_str(total), "$", -1)) < 0)
    {
      mg_http_reply(c, 500, "", "offset and total not set\n");
    }
  else if (ofs == 0 && mg_ota_begin((size_t) tot) == false)
    {
      mg_http_reply(c, 500, "", "mg_ota_begin(%ld) failed\n", tot);
    }
  else if (data.len > 0 && mg_ota_write(data.buf, data.len) == false)
    {
      mg_http_reply(c, 500, "", "mg_ota_write(%lu) @%ld failed\n", data.len, ofs);
      mg_ota_end();
    }
  else if (data.len == 0 && mg_ota_end() == false)
    {
      mg_http_reply(c, 500, "", "mg_ota_end() failed\n", tot);
    }
  else
    {
      mg_http_reply(c, 200, s_json_header, "true\n");

      if (data.len == 0)
        {

          /* Successful mg_ota_end() called, schedule device reboot */

          mg_timer_add(c->mgr, 500, 0, (void (*)(void *)) mg_device_reset, NULL);
        }
    }
}

/****************************************************************************
 * Name: handle_firmware_commit
 ****************************************************************************/

static void handle_firmware_commit(struct mg_connection *c)
{
  mg_http_reply(c, 200, s_json_header, "%s\n",
                mg_ota_commit() ? "true" : "false");
}

/****************************************************************************
 * Name: handle_firmware_rollback
 ****************************************************************************/

static void handle_firmware_rollback(struct mg_connection *c)
{
  mg_http_reply(c, 200, s_json_header, "%s\n",
                mg_ota_rollback() ? "true" : "false");
}

/****************************************************************************
 * Name: print_status
 ****************************************************************************/

static size_t print_status(void (*out)(char, void *), void *ptr, va_list *ap)
{
  int fw = va_arg(*ap, int);

  return mg_xprintf(out, ptr, "{%m:%d,%m:%c%lx%c,%m:%u,%m:%u}\n",
                    MG_ESC("status"),     mg_ota_status(fw),
                    MG_ESC("crc32"), '"', mg_ota_crc32(fw), '"',
                    MG_ESC("size"),       mg_ota_size(fw),
                    MG_ESC("timestamp"),  mg_ota_timestamp(fw));
}

/****************************************************************************
 * Name: handle_firmware_status
 ****************************************************************************/

static void handle_firmware_status(struct mg_connection *c)
{
  mg_http_reply(c, 200, s_json_header, "[%M,%M]\n", print_status,
                MG_FIRMWARE_CURRENT, print_status, MG_FIRMWARE_PREVIOUS);
}

/****************************************************************************
 * Name: handle_device_reset
 ****************************************************************************/

static void handle_device_reset(struct mg_connection *c)
{
  mg_http_reply(c, 200, s_json_header, "true\n");
  mg_timer_add(c->mgr, 500, 0, (void (*)(void *)) mg_device_reset, NULL);
}

/****************************************************************************
 * Name: handle_device_eraselast
 ****************************************************************************/

static void handle_device_eraselast(struct mg_connection *c)
{
  mg_http_reply(c, 200, s_json_header, "true\n");
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

  /* Full HTTP request/response */

  else if (ev == MG_EV_HTTP_MSG)
    {
      struct mg_http_message *hm = (struct mg_http_message *) ev_data;
      struct user_s *u = authenticate(hm);

      if (mg_match(hm->uri, mg_str("/api/#"), NULL) && (u == NULL))
        {
          mg_http_reply(c, 403, "", "Not Authorised\n");
        }
      else if (mg_match(hm->uri, mg_str("/api/login"), NULL))
        {
          handle_login(c, u);
        }
      else if (mg_match(hm->uri, mg_str("/api/logout"), NULL))
        {
          handle_logout(c);
        }
      else if (mg_match(hm->uri, mg_str("/api/debug"), NULL))
        {
          handle_debug(c, hm);
        }
      else if (mg_match(hm->uri, mg_str("/api/battery"), NULL))
        {
          handle_battery(c, hm);
        }
      else if (mg_match(hm->uri, mg_str("/api/firmware/upload"), NULL))
        {
          handle_firmware_upload(c, hm);
        }
      else if (mg_match(hm->uri, mg_str("/api/firmware/commit"), NULL))
        {
          handle_firmware_commit(c);
        }
      else if (mg_match(hm->uri, mg_str("/api/firmware/rollback"), NULL))
        {
          handle_firmware_rollback(c);
        }
      else if (mg_match(hm->uri, mg_str("/api/firmware/status"), NULL))
        {
          handle_firmware_status(c);
        }
      else if (mg_match(hm->uri, mg_str("/api/device/reset"), NULL))
        {
          handle_device_reset(c);
        }
      else if (mg_match(hm->uri, mg_str("/api/device/eraselast"), NULL))
        {
          handle_device_eraselast(c);
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
 * Name: system_daemon
 ****************************************************************************/

static int system_daemon(int argc, FAR char *argv[])
{
#if defined(CONFIG_NSH_NETINIT) && defined(CONFIG_NET_TCP)
  struct httpd_data_s *priv = &g_httpd_data;
  struct sigaction act;
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
  printf("System service [%d] started\n", g_httpd_serv.pid);

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
          break;

        default:
          goto errout;
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
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: systemd_main
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
      printf("Waiting for System service [%d] to stop\n", g_httpd_serv.pid);
      return EXIT_FAILURE;
    }

  if (!g_httpd_serv.running)
    {
      printf("Starting the System service\n");
      g_httpd_serv.pid = task_create("system_service",
                                    CONFIG_EXAMPLES_MONGOOSE_PRIORITY,
                                    CONFIG_EXAMPLES_MONGOOSE_STACKSIZE,
                                    system_daemon, argv + 1);
      if (g_httpd_serv.pid < 0)
        {
          printf("Failed to start the System service: %d\n", errno);
          return EXIT_FAILURE;
        }
    }
  else
    {
      printf("System service [%d] is running\n", g_httpd_serv.pid);
    }

  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: systemd_stop
 ****************************************************************************/

int systemd_stop(int argc, FAR char *argv[])
{
  if (!g_httpd_serv.initialized || !g_httpd_serv.running)
    {
      printf("The System service not running\n");
      return EXIT_FAILURE;
    }

  printf("Stopping the System service, pid=%d\n", g_httpd_serv.pid);
  g_httpd_serv.stop = true;

  return EXIT_SUCCESS;
}
