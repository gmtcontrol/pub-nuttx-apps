#ifndef __NETWORK_STATUS_H__
#define __NETWORK_STATUS_H__

#include <uORB/uORB.h>

/* uORB topic(s) data structure */

struct network_status_s
{
  struct
  {
    uint16_t link;
  } eth0;

  struct
  {
    uint16_t link;
  } wlan0;

  struct
  {
    uint16_t link;
  } wlan1;
};

/* uORB decleration */

ORB_DECLARE(network_status);

#endif // __NETWORK_STATUS_H__
