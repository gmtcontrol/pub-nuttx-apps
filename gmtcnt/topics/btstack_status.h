#ifndef __BTSTACK_STATUS_H__
#define __BTSTACK_STATUS_H__

#include <uORB/uORB.h>

/* uORB topic(s) data structure */

struct btstack_status_s 
{
  uint32_t state;
  uint16_t rfcid;
  uint16_t rfmtu;
};

/* uORB decleration */

ORB_DECLARE(btstack_status);

#endif // __BTSTACK_STATUS_H__
