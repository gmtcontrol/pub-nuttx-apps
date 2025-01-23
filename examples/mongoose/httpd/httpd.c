/****************************************************************************
 * apps/examples/mongoose/mongoose_main.c
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

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <debug.h>

#include <net/if.h>
#include <netinet/in.h>

#include <netutils/netlib.h>

/* Include mongoose definitions */

#include "net.h"

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern int uorb_main(FAR char *argv[]);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLINK_PERIOD_MS 1000  // LED blinking period in millis

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * main program
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
#ifndef CONFIG_NSH_NETINIT
  /* We are running standalone (as opposed to a NSH built-in app). Therefore
   * we need to initialize the network before we start.
   */

  struct in_addr addr;

  /* Set up our host address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_MONGOOSE_IPADDR);
  netlib_set_ipv4addr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_MONGOOSE_DRIPADDR);
  netlib_set_dripv4addr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_MONGOOSE_NETMASK);
  netlib_set_ipv4netmask("eth0", &addr);

  /* New versions of netlib_set_ipvXaddr will not bring the network up,
   * So ensure the network is really up at this point.
   */

  netlib_ifup("eth0");
#endif /* CONFIG_NSH_NETINIT */

#ifdef CONFIG_NET_TCP
  printf("Starting Web Server\n");

  /* Event manager */

  struct mg_mgr mgr;

  /* Set log level */

  mg_log_set(MG_LL_NONE);

  /* Initialise event manager */

  mg_mgr_init(&mgr);

  /* Initialise application */

  net_init(&mgr);

  /* Infinite event loop */

  while (1)
    {
      mg_mgr_poll(&mgr, 1000);
    }

  /* free the allocated resources */

  mg_mgr_free(&mgr);
#endif

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
  fflush(stdout);

#endif /* CONFIG_NSH_NETINIT */

  return 0;
}
