/****************************************************************************
 * apps/gmtcnt/fpga/fpga_main.c
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
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>

#include <uORB/uORB.h>
#include <system_status.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_fpga_loader_started = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * fpga_loader_main - Load a bitstream into the FPGA
 *
 * DESCRIPTION:
 *   This program loads a bitstream into the FPGA.  It takes two options:
 *
 *     -b <bitstream_name> Specify the name of the bitstream file to load.
 *     -d <device_name>    Specify the name of the FPGA device to load the
 *                          bitstream into.  If not specified, the default
 *                          device name is used.
 *
 ****************************************************************************/

static int fpga_loader(int s_argc, char **s_argv)
{
	struct system_status_s sys_stat;
	static char f_name[PATH_MAX]={0};
	char d_name[16]={0};
  int opt, len;
	int fd_dev=-1;
	int fd_bit=-1;
	char *buf;

  /* Indicate that we are running */

  g_fpga_loader_started = true;
  printf("fpga_load: Running\n");

	/* Parse command line options */

  while ((opt = getopt(s_argc, &s_argv[1], "b:d:")) != EOF)
    {
      switch (opt)
        {
					/* Get bitstream file name*/

          case 'b':
						strlcpy(f_name, optarg, sizeof(f_name));
            break;

					/* Get FPGA device path */

          case 'd':
						strlcpy(d_name, optarg, sizeof(d_name));
            break;

          default:
            printf("Usage: %s [-b <bitstream_name> -d <device_name>]\n", s_argv[1]);
            goto errout;
        }
    }

	/* If no bitstream name was specified, use the default */

	if (f_name[0] == '\0')
		{
      printf("fpga_load: missing bitstream file name\n");
      goto errout;
		}

	/* If no device name was specified, use the default */

	if (d_name[0] == '\0')
		{
			/* Use the default device name */

			strlcpy(d_name, CONFIG_GMTCNT_FPGA_DEVNAME, sizeof(d_name));
		}

  /* Open the FPGA device for writing */

  fd_dev = open(d_name, O_WRONLY);
  if (fd_dev < 0)
    {
      printf("fpga_load: open %s failed: %d\n", d_name,
																								errno);
      goto errout;
    }

  /* Open the bitstream file for reading */

  fd_bit = open(f_name, O_RDONLY);
  if (fd_bit < 0)
    {
      printf("fpga_load: open %s failed: %d\n", f_name,
																								errno);
			goto errout;
    }

	/* Allocate a buffer */

	buf = malloc(CONFIG_GMTCNT_FPGA_BUFSIZE);
	if (buf == NULL)
		{
			goto errout;
		}

	/* Read from the bitstream file */

	while ((len = read(fd_bit, buf, CONFIG_GMTCNT_FPGA_BUFSIZE)) > 0)
		{
			/* Write the bitstream to the FPGA */

			if (write(fd_dev, buf, len) != len)
				{
					printf("fpga_load: write failed: %d\n", errno);
					break;
				}
		}

	/* Free the buffer */

	free(buf);

	/* Close the file descriptors */

errout:
	close(fd_bit);

	/* Bitstream loaded successfully */

	if (close(fd_dev) == OK)
		{
			/* Create the publish file for system status */

			int pub = orb_advertise(ORB_ID(system_status), 0);
			memset(&sys_stat, 0, sizeof(sys_stat));

      /* Check the publish topic validity */

      if (pub >= 0)
        {
					/* Update system status */

					sys_stat.fpga_stat = 1;

          /* Publish the data */

          orb_publish(ORB_ID(system_status), pub, &sys_stat);

					/* Close the publisher */

					orb_close(pub);
        }
		}

	/* Indicate that we are no longer running */

  g_fpga_loader_started = false;
  printf("fpga_load: Terminating\n");

  return OK;
}

/****************************************************************************
 * @brief Main entry point for the FPGA loader application.
 *
 * This function initializes and starts the FPGA loader task. It first checks
 * if the loader is already running; if so, it exits gracefully. If not, it
 * attempts to create a new task for the FPGA loader. Upon failure to create
 * the task, it reports an error and exits with failure status. Otherwise, it
 * reports success and exits with a success status.
 *
 * @param argc The number of command line arguments.
 * @param argv The array of command line arguments.
 * @return Returns EXIT_SUCCESS if the FPGA loader is successfully started,
 *         or EXIT_FAILURE if the task creation fails.
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

	/* Print the name of the application */

  printf("fpga_load: Starting the FPGA loader\n");
  if (g_fpga_loader_started)
    {
      printf("fpga_load: FPGA loader already running\n");
      return EXIT_SUCCESS;
    }

	/* Start the task */

  ret = task_create("fpga_loader",
                    CONFIG_GMTCNT_FPGA_PRIORITY,
                    CONFIG_GMTCNT_FPGA_STACKSIZE,
                    fpga_loader,
                    argv);
  if (ret < 0)
    {
      int errcode = errno;
      printf("fpga_load: ERROR: Failed to start FPGA loader: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("fpga_load: FPGA loader started\n");
  return EXIT_SUCCESS;
}
