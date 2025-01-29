#ifndef __SYSTEM_STATUS_H__
#define __SYSTEM_STATUS_H__

#include <uORB/uORB.h>

/* uORB topic(s) data structure */

struct system_status_s 
{
	uint16_t fpga_stat;
};

/* uORB decleration */

ORB_DECLARE(system_status);

#endif // __SYSTEM_STATUS_H__
