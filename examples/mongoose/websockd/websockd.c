/****************************************************************************
 * apps/examples/mongoose/mg_websock.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAL_FS	 			    (&mg_fs_posix)
#define HAL_ROOT_DIR 		  "/"
#define HAL_WEB_ROOT_DIR 	"/sdc/web_sock"

#define HTTP_URL          "http://0.0.0.0:8080"

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct mg_connection *g_ws_conn;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * This RESTful server implements the following endpoints:
 *  /websocket - upgrade to Websocket, and implement websocket echo server
 *  /rest      - respond with JSON string {"result": 123}
 *
 * any other URI serves static files from s_web_root
 */

static void ev_handler(struct mg_connection *c, int ev, void *ev_data)
{
  /* Connection created  */
  if (ev == MG_EV_OPEN)
    {
      /* Reset the WS connection node */

      g_ws_conn = NULL;
		}
  /* Connection closed */
  else if (ev == MG_EV_CLOSE)
    {
      /* Reset the WS connection node */

      g_ws_conn = NULL;
		}
  /* Full HTTP request/response */
  else if (ev == MG_EV_HTTP_MSG)
    {
	struct mg_http_message *hm = (struct mg_http_message *) ev_data;
      if (mg_match(hm->uri, mg_str("/websocket"), NULL))
		{
					/* Upgrade to websocket. From now on, a connection is a full-duplex */
					/* Websocket connection, which will receive MG_EV_WS_MSG events. */
		mg_ws_upgrade(c, hm, NULL);
		}
			else if (mg_match(hm->uri, mg_str("/rest"), NULL))
				{
		/* Serve REST response */
		mg_http_reply(c, 200, "", "{\"result\": %d}\n", 123);
		}
			else
			{
	struct mg_http_serve_opts opts;
	memset(&opts, 0, sizeof(opts));
	opts.fs = HAL_FS;
	opts.root_dir = HAL_WEB_ROOT_DIR;
	mg_http_serve_dir(c, ev_data, &opts);
	}
	  }
  /* Websocket handshake done */
	else if (ev == MG_EV_WS_OPEN)
    {
      /* Save the connection */

      g_ws_conn = c;
    }
  /* Websocket msg, text or bin */
	else if (ev == MG_EV_WS_MSG)
		{
	/* Got websocket frame. Received data is wm->data. Echo it back! */

	struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
	mg_ws_send(c, wm->data.buf, wm->data.len, WEBSOCKET_OP_TEXT);
	}
}

/****************************************************************************
 * main program
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
#if defined(CONFIG_NSH_NETINIT) && defined(CONFIG_NET_TCP) && defined(CONFIG_NET_UDP)
  struct sockaddr_in server;
  struct sockaddr_in client;
  unsigned char inbuf[256];
  socklen_t addrlen;
  socklen_t recvlen;
  int sockfd;
  int nbytes;
  int optval;

  printf("Starting Websocket Server\n");

  /* Create a new UDP socket */

  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      printf("server: socket failure: %d\n", errno);
      exit(1);
    }

  /* Set socket to reuse address */

  optval = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval,
                 sizeof(int)) < 0)
    {
      printf("server: setsockopt SO_REUSEADDR failure: %d\n", errno);
      exit(1);
    }

  server.sin_family      = AF_INET;
  server.sin_port        = HTONS(CONFIG_EXAMPLES_MONGOOSE_SERVPORTNO);
  server.sin_addr.s_addr = HTONL(INADDR_ANY);
  addrlen                = sizeof(struct sockaddr_in);

  /* Bind the socket */

  if (bind(sockfd, (struct sockaddr *)&server, addrlen) < 0)
    {
      printf("server: bind failure: %d\n", errno);
      exit(1);
    }

  /* Clear the websocket connection instance */

  g_ws_conn = NULL;

  /* Event manager */

  struct mg_mgr mgr;

  /* Set log level */

  mg_log_set(MG_LL_NONE);

  /* Initialise event manager */

  mg_mgr_init(&mgr);

  /* Create HTTP listener */

  mg_http_listen(&mgr, HTTP_URL, ev_handler, NULL);

  /* Infinite event loop */

  while (1)
    {
      /* Event manager poll */

      mg_mgr_poll(&mgr, 1000);

      /* Check the backend service data */

      recvlen = addrlen;
      nbytes = recvfrom(sockfd, inbuf, sizeof(inbuf), MSG_DONTWAIT,
                        (struct sockaddr *)&client, &recvlen);

      /* Chech the data validity */

      if (nbytes > 0)
        {
          /* Send the received data to the client */

          if (g_ws_conn != NULL)
            {
              mg_ws_send(g_ws_conn, inbuf, nbytes, WEBSOCKET_OP_TEXT);
            }
        }
    }

  /* Close the service socket */

  close(sockfd);

  /* free the allocated resources */

  mg_mgr_free(&mgr);
#endif /* CONFIG_NSH_NETINIT && CONFIG_NET_TCP && CONFIG_NET_UDP */

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
