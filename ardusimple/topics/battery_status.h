#ifndef __BATTERY_STATUS_H__
#define __BATTERY_STATUS_H__

#include <uORB/uORB.h>

/* uORB topic(s) data structure */

struct battery_status_s
{
  int  state;
  int  health;
  int  voltage;
  int  percent;
  bool cable;
};

/* uORB decleration */

ORB_DECLARE(battery_status);

#endif // __BATTERY_STATUS_H__
