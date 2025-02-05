/****************************************************************************
 * apps/gmtcnt/netlink/netlink.c
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
#include <nuttx/net/mii.h>
#include <nuttx/ioexpander/gpio.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <net/if.h>
#include <netinet/in.h>

#include <uORB/uORB.h>
#include <network_status.h>

#include "netutils/netlib.h"

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern int systemd_start_main (int argc, FAR char *argv[]);
extern int systemd_stop_main  (int argc, FAR char *argv[]);

extern int ftpd_start_main    (int argc, FAR char *argv[]);
extern int ftpd_stop_main     (int argc, FAR char *argv[]);

extern int dhcpd_start_main		(int argc, FAR char *argv[]);
extern int dhcpd_stop_main 		(int argc, FAR char *argv[]);

extern int renew_main					(int argc, FAR char *argv[]);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LONG_TIME_SEC  	(2)    	/* 2 seconds */
#define SHORT_TIME_SEC  (2)    	/* 2 seconds */

#define VBUS_DEVPATH 		"/dev/gpio0"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
static sem_t g_notify_sem;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_signal
 *
 * Description:
 *   This signal handler responds to changes in PHY status.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
static void netlink_signal(int signo, FAR siginfo_t *siginfo,
                           FAR void *context)
{
  int semcount;
  int ret;

  /* What is the count on the semaphore?  Don't over-post */

  ret = sem_getvalue(&g_notify_sem, &semcount);
  ninfo("Entry: semcount=%d\n", semcount);

  if (ret == OK && semcount <= 0)
    {
      sem_post(&g_notify_sem);
    }

  ninfo("Exit\n");
}
#endif

/****************************************************************************
 * Name: netlink_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct timespec abstime;
  struct timespec reltime;
  struct sigaction act;
  struct sigaction oact;
  struct ifreq ifr;

  struct network_status_s net_stat;
  struct in_addr addr;
  int fd, sd, ret;
  int pub;

  ninfo("Entry\n");

  /* Initialize the notification semaphore */

#ifdef CONFIG_ARCH_PHY_INTERRUPT
  DEBUGVERIFY(sem_init(&g_notify_sem, 0, 0));
#endif

  /* Get a socket descriptor that we can use to communicate with the network
   * interface driver.
   */

  sd = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE, NET_SOCK_PROTOCOL);
  if (sd < 0)
    {
      ret = -errno;
      DEBUGASSERT(ret < 0);

      nerr("ERROR: Failed to create a socket: %d\n", ret);
      goto errout;
    }

  /* open the VBUS GPIO */

  fd = open(VBUS_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      ret = -errno;
      DEBUGASSERT(ret < 0);

      nerr("ERROR: Failed to open VBUS gpio: %d\n", ret);
      goto errout_with_socket;
    }

#ifdef CONFIG_ARCH_PHY_INTERRUPT
  /* Attach a signal handler so that we do not lose PHY events */

  act.sa_sigaction = netlink_signal;
  act.sa_flags = SA_SIGINFO;

  ret = sigaction(CONFIG_GMTCNT_NETLINK_SIGNO, &act, &oact);
  if (ret < 0)
    {
      ret = -errno;
      DEBUGASSERT(ret < 0);

      nerr("ERROR: sigaction() failed: %d\n", ret);
      goto errout_with_socket;
    }
#endif /* CONFIG_ARCH_PHY_INTERRUPT */

  /* Create the publish file for network status */

  pub = orb_advertise(ORB_ID(network_status), 0);
	memset(&net_stat, 0, sizeof(net_stat));

  /* Now loop, waiting for changes in link status */

  bool eth0link = false;
  bool eth1link = false;
  bool eth1serv = false;
	bool dhcpd 		= false;
	bool vbus;

  while (1)
    {
			/* Configure to receive a signal on changes in link status */

			memset(&ifr, 0, sizeof(struct ifreq));
			strlcpy(ifr.ifr_name, "eth0", IFNAMSIZ);

#ifdef CONFIG_ARCH_PHY_INTERRUPT
			ifr.ifr_mii_notify_event.sigev_notify = SIGEV_SIGNAL;
			ifr.ifr_mii_notify_event.sigev_signo  = CONFIG_GMTCNT_NETLINK_SIGNO;

			/* Notify the driver that we want link status changes */

			ret = ioctl(sd, SIOCMIINOTIFY, (unsigned long)&ifr);
			if (ret < 0)
				{
					ret = -errno;
					DEBUGASSERT(ret < 0);

					nerr("ERROR: ioctl(SIOCMIINOTIFY) failed: %d\n", ret);
					goto errout_with_sigaction;
				}
#endif /* CONFIG_ARCH_PHY_INTERRUPT */

      /* Does the driver think that the link is up or down? */

      ret = ioctl(sd, SIOCGIFFLAGS, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          nerr("ERROR: ioctl(SIOCGIFFLAGS) failed: %d\n", ret);
          goto errout_with_notification;
        }

      eth0link = ((ifr.ifr_flags & IFF_UP) != 0);

      /* Get the current PHY address in use.  This probably does not change,
       * but just in case...
       *
       * NOTE: We are assuming that the network device name is preserved in
       * the ifr structure.
       */

      ret = ioctl(sd, SIOCGMIIPHY, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          nerr("ERROR: ioctl(SIOCGMIIPHY) failed: %d\n", ret);
          goto errout_with_notification;
        }

      /* Read the PHY status register */

      ifr.ifr_mii_reg_num = MII_MSR;
      ret = ioctl(sd, SIOCGMIIREG, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          nerr("ERROR: ioctl(SIOCGMIIREG) failed: %d\n", ret);
          goto errout_with_notification;
        }

      ninfo("%s: eth0link=%d PHY address=%02x MSR=%04x\n",
            ifr.ifr_name, eth0link, ifr.ifr_mii_phy_id, ifr.ifr_mii_val_out);

      /* Check for link up or down */

      if ((ifr.ifr_mii_val_out & MII_MSR_LINKSTATUS) != 0)
        {
          /* Link up... does the drive think that the link is up? */

          if (!eth0link)
            {
              /* No... We just transitioned from link down to link up.
               * Bring the link up.
               */

              ninfo("Bringing the link up\n");

              ifr.ifr_flags = IFF_UP;
              ret = ioctl(sd, SIOCSIFFLAGS, (unsigned long)&ifr);
              if (ret < 0)
                {
                  ret = -errno;
                  DEBUGASSERT(ret < 0);

                  nerr("ERROR: ioctl(SIOCSIFFLAGS) failed: %d\n", ret);
                  goto errout_with_notification;
                }

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
              /* Perform ICMPv6 auto-configuration */

              netlib_icmpv6_autoconfiguration(ifr.ifr_name);
#endif
              /* And wait for a short delay.  We will want to recheck the
               * link status again soon.
               */

              reltime.tv_sec  = SHORT_TIME_SEC;
              reltime.tv_nsec = 0;

							/* Renew the IP address */

							FAR char *renew_argv[]=
							{
								"renew",
								"eth0",
								NULL
							};
							renew_main(2, renew_argv);
            }
          else
            {
              /* The link is still up.  Take a long, well-deserved rest */

#ifdef CONFIG_ARCH_PHY_INTERRUPT
              reltime.tv_sec  = LONG_TIME_SEC;
#else
							reltime.tv_sec  = SHORT_TIME_SEC;
#endif /* CONFIG_ARCH_PHY_INTERRUPT */

              reltime.tv_nsec = 0;
            }
        }
      else
        {
          /* Link down... Was the driver link state already down? */

          if (eth0link)
            {
              /* No... we just transitioned from link up to link down.  Take
               * the link down.
               */

              ninfo("Taking the link down\n");

              ifr.ifr_flags = 0;
              ret = ioctl(sd, SIOCSIFFLAGS, (unsigned long)&ifr);
              if (ret < 0)
                {
                  ret = -errno;
                  DEBUGASSERT(ret < 0);

                  nerr("ERROR: ioctl(SIOCSIFFLAGS) failed: %d\n", ret);
                  goto errout_with_notification;
                }
            }

          /* In either case, wait for the short, configurable delay */

          reltime.tv_sec  = CONFIG_GMTCNT_NETLINK_RETRYMSEC / 1000;
          reltime.tv_nsec = (CONFIG_GMTCNT_NETLINK_RETRYMSEC % 1000) * 1000000;
        }

      /* Now wait for either the semaphore to be posted for a timed-out to
       * occur.
       */

      sched_lock();
      DEBUGVERIFY(clock_gettime(CLOCK_REALTIME, &abstime));

      abstime.tv_sec  += reltime.tv_sec;
      abstime.tv_nsec += reltime.tv_nsec;
      if (abstime.tv_nsec >= 1000000000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000000000;
        }

			/* Wait for the semaphore to be posted */

      sem_timedwait(&g_notify_sem, &abstime);
      sched_unlock();

			/* Read the VBUS status */

			ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&vbus));

			/* Check link status of the interface */

			if ((ret == OK) && (vbus != eth1link))
				{
					/* Update the link status */

					if ((eth1link = vbus) != 0)
						{
							/* Bring the network up */

							netlib_ifup("eth1");
						} 
					else 
						{
							/* Bring the network down */

							netlib_ifdown("eth1");
						}

					/* Get the assigned IP address to the interface */

					netlib_get_ipv4addr("eth1", &addr);

					/* Has the ethernet services started ? */

					if (!eth1serv)
						{
							/* Link is up ? */

							if (eth1link && (addr.s_addr != 0))
								{
									/* Local DHCP service */

									if (!dhcpd)
										{
											FAR char *dhcpd_argv[]=
											{
												"dhcpd_start",
												"eth1",
												NULL
											};

											dhcpd_start_main(2, dhcpd_argv);
											dhcpd = true;
										}	

									/* Local FTP service */

									FAR char *ftpd_argv[]=
									{
										"ftpd_start", 
										"-i", "eth1",
										"-4",
										NULL
									};
									ftpd_start_main(4, ftpd_argv);

									/* Local system service */

									FAR char *systemd_argv[]=
									{
										"systemd_start",
										"-p", "8181",
										"-r", "/data0/www_root",
										"-h", inet_ntoa(addr),
										NULL
									};
									systemd_start_main(7, systemd_argv);

									/* Ethernet services started */

									eth1serv = true;
								}
						}
					else
						{
							/* Check for link down state */

							if (!eth1link)
								{
									/* Stop all services */

									ftpd_stop_main(0, NULL);
									systemd_stop_main(0, NULL);
									eth1serv = false;
								}
						}
				}

      /* Prepare the uORB data */

      net_stat.eth0.link  = eth0link;
      net_stat.eth1.link  = eth1link;

      /* Check the publish topic validity */

      if (pub >= 0)
        {
          /* Publish the data */

          orb_publish(ORB_ID(network_status), pub, &net_stat);
        }
    }

errout_with_notification:
  ifr.ifr_mii_notify_event.sigev_notify = SIGEV_NONE;
  ioctl(sd, SIOCMIINOTIFY, (unsigned long)&ifr);
errout_with_sigaction:
  sigaction(CONFIG_GMTCNT_NETLINK_SIGNO, &oact, NULL);
errout_with_socket:
  close(sd);
  close(fd);
errout:
  nerr("ERROR: Aborting\n");
  return ret;
}
