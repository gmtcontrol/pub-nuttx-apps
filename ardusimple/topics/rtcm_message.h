#ifndef __RTCM_MESSAGE_H__
#define __RTCM_MESSAGE_H__

#include <uORB/uORB.h>

#define RTCM_MESSAGE_SIZE   (1024)

/* uORB topic(s) data structure */

struct rtcm_message_s
{
  uint32_t len;
  uint8_t *msg;
};

/* uORB declerations */

ORB_DECLARE(rtcm_message);

#endif // __RTCM_MESSAGE_H__
