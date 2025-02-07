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

#include <sys/ioctl.h>
#include <sys/boardctl.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>

#include "netutils/netlib.h"

#ifdef CONFIG_NET_6LOWPAN
#  include <nuttx/net/sixlowpan.h>
#endif

#ifdef CONFIG_NET_IEEE802154
#  include <nuttx/net/ieee802154.h>
#endif

#ifdef CONFIG_NETUTILS_NTPCLIENT
#  include "netutils/ntpclient.h"
#endif

#if defined(CONFIG_FSUTILS_IPCFG)
#  include "fsutils/ipcfg.h"
#endif

#include "netutils/netinit.h"

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DHCPC
static bool g_use_dhcpc;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netinit_set_macaddr
 *
 * Description:
 *   Set the hardware MAC address if the hardware is not capable of doing
 *   that for itself.
 *
 ****************************************************************************/

#if defined(CONFIG_NETINIT_NOMAC) && defined(HAVE_MAC)
static void netinit_set_macaddr(FAR const char *ifname)
{
#if defined(CONFIG_NETINIT_UIDMAC)
  uint8_t uid[CONFIG_BOARDCTL_UNIQUEID_SIZE];
#elif defined(CONFIG_NET_ETHERNET)
  uint8_t mac[IFHWADDRLEN];
#elif defined(HAVE_EADDR)
  uint8_t eaddr[8];
#endif

  /* Many embedded network interfaces must have a software assigned MAC */

#if defined(CONFIG_NETINIT_UIDMAC)
  boardctl(BOARDIOC_UNIQUEID, (uintptr_t)&uid);
  uid[0] = (uid[0] & 0b11110000) | 2; /* Locally Administered MAC */
  netlib_setmacaddr(ifname, uid);

#elif defined(CONFIG_NET_ETHERNET)
  /* Use the configured, fixed MAC address */

  mac[0] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 1)) & 0xff;
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;

  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;

  /* Set the MAC address */

  netlib_setmacaddr(ifname, mac);

#elif defined(HAVE_EADDR)
  /* Use the configured, fixed extended address */

  eaddr[0] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 3)) & 0xff;
  eaddr[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 2)) & 0xff;
  eaddr[2] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 1)) & 0xff;
  eaddr[3] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;

  eaddr[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  eaddr[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  eaddr[6] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  eaddr[7] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;

  /* Set the 6LoWPAN extended address */

  netlib_seteaddr(ifname, eaddr);
#endif /* CONFIG_NET_ETHERNET or HAVE_EADDR */
}
#else
#  define netinit_set_macaddr(ifname)
#endif

/****************************************************************************
 * Check if the IP configuration directory exists.  If it does not exist,
 * try repeatedly until it does or until a maximum number of retries
 * has been exceeded.
 *
 * This function is used to handle the case where the IP configuration
 * directory is created by a file system initialization script.
 *
 ****************************************************************************/

#if CONFIG_NETINIT_RETRY_MOUNTPATH > 0
static inline void netinit_checkpath(void)
{
  int retries = CONFIG_NETINIT_RETRY_MOUNTPATH;
  while (retries > 0)
    {
      DIR * dir = opendir(CONFIG_IPCFG_PATH);
      if (dir)
        {
          /* Directory exists. */

          closedir(dir);
          break;
        }
      else
        {
        usleep(100000);
        }

      retries--;
    }
}
#endif

/****************************************************************************
 * Name: netinit_set_ipv4addrs
 *
 * Description:
 *   Setup IPv4 addresses.
 *
 ****************************************************************************/

#if !defined(CONFIG_NET_6LOWPAN) && !defined(CONFIG_NET_IEEE802154) && \
     defined(CONFIG_NET_IPv4)
static inline void netinit_set_ipv4addrs(FAR const char *ifname)
{
  struct in_addr addr;
#ifdef CONFIG_FSUTILS_IPCFG
  struct ipv4cfg_s ipv4cfg;
  int ret;

  /* Attempt to obtain IPv4 address configuration from the IP configuration
   * file.
   */

#if defined(CONFIG_NETINIT_THREAD) && CONFIG_NETINIT_RETRY_MOUNTPATH > 0
  netinit_checkpath();
#endif

  ret = ipcfg_read(ifname, (FAR struct ipcfg_s *)&ipv4cfg, AF_INET);
#ifdef CONFIG_NETUTILS_DHCPC
  if (ret >= 0 && ipv4cfg.proto != IPv4PROTO_NONE)
#else
  if (ret >= 0 && IPCFG_HAVE_STATIC(ipv4cfg.proto))
#endif
    {
      /* Check if we are using DHCPC */

#ifdef CONFIG_NETUTILS_DHCPC
      if (IPCFG_USE_DHCP(ipv4cfg.proto))
        {
          g_use_dhcpc = true;
          addr.s_addr = 0;
        }
      else
#endif
        {
          /* We are not using DHCPC.  We need an IP address */

#ifdef CONFIG_NETINIT_IPADDR
          /* Check if we have a static IP address in the configuration file */

          if (IPCFG_HAVE_STATIC(ipv4cfg.proto))
            {
              addr.s_addr = ipv4cfg.ipaddr;
            }
          else
            {
              /* This is not a good option, but in this case what else can
               * we do?
               */

              addr.s_addr = HTONL(CONFIG_NETINIT_IPADDR);
            }
#else
          /* Use whatever was provided in the file (might be zero) */

          addr.s_addr = ipv4cfg.ipaddr;
#endif
        }

      netlib_set_ipv4addr(ifname, &addr);

      /* Set up the remaining addresses */

      if (IPCFG_HAVE_STATIC(ipv4cfg.proto))
        {
          /* Set up the default router address */

          addr.s_addr = ipv4cfg.router;
          netlib_set_dripv4addr(ifname, &addr);

          /* Setup the subnet mask */

          addr.s_addr = ipv4cfg.netmask;
          netlib_set_ipv4netmask(ifname, &addr);
        }

#ifdef CONFIG_NETUTILS_DHCPC
      /* No static addresses?  That is fine if we are have addresses
       * provided by the configuration, or if we are using DHCP.
       */

      else if (g_use_dhcpc)
        {
          /* Set up the default router address and sub-net mask */

          addr.s_addr = 0;
          netlib_set_dripv4addr(ifname, &addr);
          netlib_set_ipv4netmask(ifname, &addr);
        }
#endif
      else
        {
          /* Otherwise, set up the configured default router address */

          addr.s_addr = HTONL(CONFIG_NETINIT_DRIPADDR);
          netlib_set_dripv4addr(ifname, &addr);

          /* Setup the subnet mask */

          addr.s_addr = HTONL(CONFIG_NETINIT_NETMASK);
          netlib_set_ipv4netmask(ifname, &addr);
        }

#ifdef CONFIG_NETINIT_DNS
      /* Set up the DNS address.  Was one provided in the configuration? */

      if (ipv4cfg.dnsaddr == 0)
        {
          /* No, use the configured default */

          addr.s_addr = HTONL(CONFIG_NETINIT_DNSIPADDR);
        }
      else
        {
          addr.s_addr = ipv4cfg.dnsaddr;
        }

      netlib_set_ipv4dnsaddr(&addr);
#endif
    }
  else
#endif
    {
      /* Set up our host address */

#ifdef CONFIG_NETINIT_DHCPC
      g_use_dhcpc = true;
      addr.s_addr = 0;
#else
      addr.s_addr = HTONL(CONFIG_NETINIT_IPADDR);
#endif
      netlib_set_ipv4addr(ifname, &addr);

      /* Set up the default router address */

      addr.s_addr = HTONL(CONFIG_NETINIT_DRIPADDR);
      netlib_set_dripv4addr(ifname, &addr);

      /* Setup the subnet mask */

      addr.s_addr = HTONL(CONFIG_NETINIT_NETMASK);
      netlib_set_ipv4netmask(ifname, &addr);

#ifdef CONFIG_NETINIT_DNS
      addr.s_addr = HTONL(CONFIG_NETINIT_DNSIPADDR);
      netlib_set_ipv4dnsaddr(&addr);
#endif
    }
}
#endif

/****************************************************************************
 * Name: netinit_set_ipv6addrs
 *
 * Description:
 *   Setup IPv6 addresses.
 *
 ****************************************************************************/

#if !defined(CONFIG_NET_6LOWPAN) && !defined(CONFIG_NET_IEEE802154) && \
     defined(CONFIG_NET_IPv6)
static inline void netinit_set_ipv6addrs(FAR const char *ifname)
{
#ifndef CONFIG_NET_ICMPv6_AUTOCONF
#ifdef CONFIG_FSUTILS_IPCFG
  struct ipv6cfg_s ipv6cfg;
  int ret;
#endif

#ifdef CONFIG_FSUTILS_IPCFG
  /* Attempt to obtain IPv6 address configuration from the IP configuration
   * file.
   */

#if defined(CONFIG_NETINIT_THREAD) && CONFIG_NETINIT_RETRY_MOUNTPATH > 0
  netinit_checkpath();
#endif

  ret = ipcfg_read(ifname, (FAR struct ipcfg_s *)&ipv6cfg, AF_INET6);
  if (ret >= 0 && IPCFG_HAVE_STATIC(ipv6cfg.proto))
    {
      /* Set up our fixed host address */

      netlib_set_ipv6addr(ifname, &ipv6cfg.ipaddr);

      /* Set up the default router address */

      netlib_set_dripv6addr(ifname, &ipv6cfg.router);

      /* Setup the subnet mask */

      netlib_set_ipv6netmask(ifname, &ipv6cfg.netmask);
    }
  else
#endif
    {
      /* Set up our fixed host address */

      netlib_set_ipv6addr(ifname,
                          (FAR const struct in6_addr *)g_ipv6_hostaddr);

      /* Set up the default router address */

      netlib_set_dripv6addr(ifname,
                            (FAR const struct in6_addr *)g_ipv6_draddr);

      /* Setup the subnet mask */

      netlib_set_ipv6netmask(ifname,
                            (FAR const struct in6_addr *)g_ipv6_netmask);
    }
#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
}
#endif

/****************************************************************************
 * Name: netinit_set_ipaddrs
 *
 * Description:
 *   Setup IP addresses.
 *
 *   For 6LoWPAN, the IP address derives from the MAC address.  Setting it
 *   to any user provided value is asking for trouble.
 *
 ****************************************************************************/

#if !defined(CONFIG_NET_6LOWPAN) && !defined(CONFIG_NET_IEEE802154)
static void netinit_set_ipaddrs(FAR const char *ifname)
{
#ifdef CONFIG_NET_IPv4
  netinit_set_ipv4addrs(ifname);
#endif

#ifdef CONFIG_NET_IPv6
  netinit_set_ipv6addrs(ifname);
#endif
}
#else
#  define netinit_set_ipaddrs(ifname)
#endif

/****************************************************************************
 * Name: netinit_net_bringup()
 *
 * Description:
 *   Bring up the configured network
 *
 ****************************************************************************/

#if !defined(CONFIG_NETINIT_NETLOCAL)
static int netinit_net_bringup(FAR const char *ifname)
{
  /* Bring the network up. */

  if (netlib_ifup(ifname) < 0)
    {
      return ERROR;
    }

#if defined(CONFIG_WIRELESS_WAPI) && defined(CONFIG_DRIVERS_IEEE80211)
  /* Associate the wlan with an access point. */

  if (netinit_associate(ifname) < 0)
    {
      return ERROR;
    }
#endif

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Perform ICMPv6 auto-configuration */

  netlib_icmpv6_autoconfiguration(ifname);
#endif

#ifdef CONFIG_NETUTILS_DHCPC
  if (g_use_dhcpc)
    {
      if (netlib_obtain_ipv4addr(ifname) < 0)
        {
          return ERROR;
        }
    }
#endif

#ifdef CONFIG_NETUTILS_NTPCLIENT
  /* Start the NTP client */

  ntpc_start();
#endif

  return OK;
}
#else
#  define netinit_net_bringup(ifname)   (OK)
#endif

/****************************************************************************
 * Name: netinit_configure
 *
 * Description:
 *   Initialize the network per the selected NuttX configuration
 *
 ****************************************************************************/

static void netinit_configure(void)
{
  /* Many embedded network interfaces must have a software assigned MAC */

  netinit_set_macaddr("eth0");

  /* Use WiFi module MAC address */

  /* Set up IP addresses */

  netinit_set_ipaddrs("eth0");
  netinit_set_ipaddrs("wlan0");
  netinit_set_ipaddrs("wlan1");
#ifdef CONFIG_NET_BLUETOOTH
  netinit_set_ipaddrs("bnep0");
#endif /* CONFIG_NET_BLUETOOTH */

  /* That completes the 'local' initialization of the network device. */

#ifndef CONFIG_NETINIT_NETLOCAL
  /* Bring the network(s) up. */

  netinit_net_bringup("wlan0");
  netinit_net_bringup("wlan1");
#ifdef CONFIG_NET_BLUETOOTH
  netinit_net_bringup("bnep0");
#endif /* CONFIG_NET_BLUETOOTH */
#endif /* CONFIG_NETINIT_NETLOCAL */
}

/****************************************************************************
 * Name: netlink_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  /* Ignore arguments */

  UNUSED(argc);
  UNUSED(argv);

  /* Initialize the network per the selected user configuration */

  netinit_configure();

  return OK;
}
