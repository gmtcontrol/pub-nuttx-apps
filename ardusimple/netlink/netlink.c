/****************************************************************************
 * apps/ardusimple/netlink/netlink.c
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
#include <nuttx/net/netconfig.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <net/if.h>
#include <netinet/in.h>

#include <uORB/uORB.h>
#include <battery_status.h>
#include <network_status.h>

#include "netutils/netlib.h"

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern int systemd_start_main (int argc, FAR char *argv[]);
extern int systemd_stop_main  (int argc, FAR char *argv[]);

extern int smartd_start_main  (int argc, FAR char *argv[]);
extern int smartd_stop_main   (int argc, FAR char *argv[]);

extern int ftpd_start_main    (int argc, FAR char *argv[]);
extern int ftpd_stop_main     (int argc, FAR char *argv[]);

extern int dhcpd_start_main   (int argc, FAR char *argv[]);
extern int dhcpd_stop_main    (int argc, FAR char *argv[]);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: netlink_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct network_status_s net_stat;
  struct battery_status_s bat_stat;
  struct in_addr addr;
  struct iwreq iwr;
  struct ifreq ifr;
  int pub, sub;
  int sd, ret;

  /* Get a socket descriptor that we can use to communicate with the network
   * interface driver.
   */

  sd = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE, NET_SOCK_PROTOCOL);
  if (sd < 0)
    {
      ret = -errno;
      DEBUGASSERT(ret < 0);

      printf("ERROR: Failed to create a socket: %d\n", ret);
      goto errout;
    }

  /* Put the driver name into the request(s) */

  memset(&iwr, 0, sizeof(struct iwreq));
  strlcpy(iwr.ifr_name, "wlan1", IFNAMSIZ);

  /* Create the publish file for network status */

  pub = orb_advertise(ORB_ID(network_status), 0);

  /* Create the subscribe file for battery status */

  sub = orb_subscribe(ORB_ID(battery_status));

  /* Now loop, waiting for changes in link status */

  bool ethlink = false;
  bool ethserv = false;
  bool wl0link = false;
  bool wl0serv = false;
  bool wl1link = false;
  bool wl1serv = false;
  bool dhcpd   = false;
  bool updated;

  while (1)
    {
      /* Check the uORB status */

      if ((orb_check(sub, &updated) == OK) && updated)
        {
          /* Get the published data */

          if (orb_copy(ORB_ID(battery_status), sub, &bat_stat) == OK)
            {
              /* Check link status of interface */

              if (bat_stat.cable != ethlink)
                {
                  if (bat_stat.cable)
                    {
                       /* Bring the network up */

                      netlib_ifup("eth0");
                    }
                  else
                    {
                       /* Bring the network down */

                      netlib_ifdown("eth0");
                    }

                  /* Update the link status */

                  ethlink = bat_stat.cable;
                }
            }
        }

      /* Has the ethernet services started ? */

      if (!ethserv)
        {
          if (ethlink)
            {
              /* Get the assigned IP address to the interface */

              netlib_get_ipv4addr("eth0", &addr);

              /* Start the local ftp service */
              #if 0
              FAR char *ftpd_argv[]=
              {
                "ftpd_start",
                "-i", "eth0",
                "-4",
                NULL
              };

              ftpd_start_main(4, ftpd_argv);
              #endif

              /* Start the local system service */
              #if 0
              FAR char *systemd_argv[]=
              {
                "systemd_start",
                "-p", "8181",
                "-r", "/data0/www_dash",
                "-h", inet_ntoa(addr),
                NULL
              };

              systemd_start_main(7, systemd_argv);
              #endif

              /* Start the local web service */
              #if 0
              FAR char *smartd_argv[]=
              {
                "smartd_start",
                "-p", "8080",
                "-r", "/data0/www_eth",
                "-h", inet_ntoa(addr),
                "-g",
                NULL
              };

              smartd_start_main(8, smartd_argv);
              #endif

              /* Ethernet services started */

              ethserv = true;
            }
        }
      else
        {
          /* Check for link down state */

          if (!ethlink)
            {
              /* Stop all services */

              ftpd_stop_main(0, NULL);
              smartd_stop_main(0, NULL);
              systemd_stop_main(0, NULL);
              ethserv = false;
            }
        }

      /* Is the "wlan0" interface up or down? */

      memset(&ifr, 0, sizeof(struct ifreq));
      strlcpy(ifr.ifr_name, "wlan0", IFNAMSIZ);

      ret = ioctl(sd, SIOCGIFFLAGS, (unsigned long)&ifr);
      if (ret == OK)
        {
          /* Get the assigned IP address to the interface */

          addr.s_addr = 0;
          if (netlib_get_ipv4addr(ifr.ifr_name, &addr) == OK)
            {
              wl0link = ((ifr.ifr_flags & (IFF_UP | IFF_RUNNING)) != 0) && (addr.s_addr != 0);
            }
        }
      else
        {
          printf("WARNING: ioctl(SIOCGIFFLAGS) failed: %d\n", -errno);
        }

      /* Get the client associate status of the "wlan1" interface */

      ret = ioctl(sd, SIOCGIWAPASSOC, (unsigned long)((uintptr_t)&iwr));
      if (ret == OK)
        {
          /* Is the "wlan1" interface up or down? */

          memset(&ifr, 0, sizeof(struct ifreq));
          strlcpy(ifr.ifr_name, "wlan1", IFNAMSIZ);

          ret = ioctl(sd, SIOCGIFFLAGS, (unsigned long)&ifr);
          if (ret == OK)
            {
              wl1link = ((ifr.ifr_flags & (IFF_UP | IFF_RUNNING)) != 0) && (iwr.u.data.length > 0);
            }
          else
            {
              printf("WARNING: ioctl(SIOCGIFFLAGS) failed: %d\n", -errno);
            }
        }

      /* Has the "wlan0" services started ? */

      if (!wl0serv)
        {
          if (wl0link)
            {
              /* Get the assigned IP address to the interface */

              netlib_get_ipv4addr("wlan0", &addr);

              /* Start the WiFi services */

              /* WiFi services started */

              wl0serv = true;
            }
        }
      else
        {
          /* Check for link down state */

          if (!wl0link)
            {
              /* Stop all services */

            }
        }

      /* Has the "wlan1" services started ? */

      if (!wl1serv)
        {
          if (wl1link)
            {
              /* Get the assigned IP address to the interface */

              netlib_get_ipv4addr("wlan1", &addr);

              /* Start the WiFi services */

              #ifdef CONFIG_SYSTEM_SYSTEM
              /* Telnet daemon */

              //system("telnetd &");
              #endif

              /* DHCP daemon */

              if (!dhcpd)
                {
                  FAR char *dhcpd_argv[]=
                  {
                    "dhcpd_start",
                    "wlan1",
                    NULL
                  };

                  dhcpd_start_main(2, dhcpd_argv);
                  dhcpd = true;
                }

              /* Remote web service */

              FAR char *smartd_argv[]=
              {
                "smartd_start",
                "-p", "80",
                "-r", "/data0/www_wlan",
                "-h", inet_ntoa(addr),
                "-g",
                NULL
              };

              smartd_start_main(8, smartd_argv);

              /* WiFi services started */

              wl1serv = true;
            }
        }
      else
        {
          /* Check for link down state */

          if (!wl1link)
            {
              /* Stop all services */

              smartd_stop_main(0, NULL);
              wl1serv = false;
            }
        }

      /* Prepare the uORB data */

      net_stat.eth0.link  = ethlink;
      net_stat.wlan0.link = wl0link;
      net_stat.wlan1.link = wl1link;

      /* Check the publish topic validity */

      if (pub >= 0)
        {
          /* Publish the data */

          orb_publish(ORB_ID(network_status), pub, &net_stat);
        }

      /* Suspend the for 500ms */

      usleep(500 * 1000);
    }

  ninfo("Exit\n");

  close(sd);
errout:
  nerr("ERROR: Aborting\n");
  return ret;
}
