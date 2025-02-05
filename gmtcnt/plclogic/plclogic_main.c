/****************************************************************************
 * apps/boot/tinyboot/plclogic_main.c
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

#include <arch/board/boardctl.h>
#include <stdio.h>
#include <syslog.h>

#include <plclogic.h>

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: plclogic_loader
 *
 * Description:
 *   PLC core logic loader.
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct boardioc_image_info_s info;
	int ret;

#if defined(CONFIG_BOARDCTL) && !defined(CONFIG_NSH_ARCHINIT)
  /* Perform architecture-specific initialization (if configured) */

  boardctl(BOARDIOC_INIT, 0);

#ifdef CONFIG_BOARDCTL_FINALINIT
  /* Perform architecture-specific final-initialization (if configured) */

  boardctl(BOARDIOC_FINALINIT, 0);
#endif
#endif

  syslog(LOG_INFO, "*** plclogic ***\n");

  /* Check the primary image confirmation */

  if (plclogic_get_confirm())
    {
      syslog(LOG_INFO, "Found executable image, run from user applicaiton slot.\n");

      /* Call board specific image boot */

			memset(&info, 0, sizeof(info));
      info.path        = CONFIG_GMTCNT_PLCLOGIC_SECONDARY_SLOT_PATH;
      info.header_size = CONFIG_GMTCNT_PLCLOGIC_HEADER_SIZE;

			/* Get the image vector table */

      ret = boardctl(BOARDIOC_OTA_GETVECTOR, (uintptr_t)&info);
			if (ret != OK)
				{
					return EXIT_FAILURE;
				}

			/* Execute the image */

			return boardctl(BOARDIOC_OTA_EXEVECTOR, (uintptr_t)&info);
    }
	else
		{
			return EXIT_FAILURE;
		}
}
