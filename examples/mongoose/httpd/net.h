// Copyright (c) 2023-2024 Cesanta Software Limited
// All rights reserved

#ifndef __NET_H__
#define __NET_H__

#include <mongoose.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_FS	 			(&mg_fs_posix)
#define HAL_ROOT_DIR 		"/"
#define HAL_WEB_ROOT_DIR 	"/sdc/web_dash"

#define HTTP_URL 			"http://0.0.0.0:8000"
#define HTTPS_URL 			"https://0.0.0.0:8443"

#define MAX_DEVICE_NAME 	40
#define MAX_EVENTS_NO 		400
#define MAX_EVENT_TEXT_SIZE 10
#define EVENTS_PER_PAGE 	20

// Event log entry
struct ui_event {
  uint8_t type, prio;
  unsigned long timestamp;
  char text[10];
};

void net_init(struct mg_mgr *mgr);

#ifdef __cplusplus
}
#endif

#endif /* __NET_H__ */
