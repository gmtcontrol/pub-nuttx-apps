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
  } eth1;

  struct
  {
    uint16_t link;
  } wlan0;

  struct
  {
    uint16_t link;
  } wlan1;

  struct
  {
    uint16_t link;
  } can0;

  struct
  {
    uint16_t link;
  } can1;
};

/* uORB decleration */

ORB_DECLARE(network_status);

#endif // __NETWORK_STATUS_H__
