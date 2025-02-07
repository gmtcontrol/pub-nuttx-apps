/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "ardusimp_spp_mapkit.c"

/*
 * ardusimp_spp_mapkit.c
 */

// *****************************************************************************
/* EXAMPLE_START(ardusimp_spp_mapkit): Performance - Stream GNSS Data over SPP (Server)
 *
 * @text After RFCOMM connections gets open, request a
 * RFCOMM_EVENT_CAN_SEND_NOW via rfcomm_request_can_send_now_event().
 * @text When we get the RFCOMM_EVENT_CAN_SEND_NOW, send data and request another one.
 *
 * @text Note: To test, run the example, pair from a remote
 * device, and open the Virtual Serial Port.
 */
// *****************************************************************************

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/param.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>

#include <nuttx/mm/circbuf.h>
#include <uORB/uORB.h>
#include <rtcm_message.h>
#include <btstack_status.h>

#include "btstack.h"

#define RFCOMM_SERVER_CHANNEL (1)

#define POLL_PERIOD_MS  (10)
#define POLL_DELAY_SEC  (1)
#define NPOLLFDS        (1)

#define GNSS_MSG_SIZE   (82)
#define GNSS_MSG_NUM    (20)

#define TEST_COD        (0x1234)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_rtcm_buf[RTCM_MESSAGE_SIZE];

struct mapkit_data_s
{
  btstack_packet_callback_registration_t hci_event;
  btstack_timer_source_t poll_timer;
  uint16_t rfcomm_cid;
  uint16_t rfcomm_mtu;
  uint8_t spp_serv_buf[150];
  struct circbuf_s circ;

  /* uORB Subscribers */

  struct
  {
    /* GNSS Raw */

    struct
    {
      struct orb_object  obj;
      FAR struct pollfd *fds;
    } gnss;

  } sub;

  /* uORB Publishers */

  struct
  {
    /* RTCM Message */

    struct
    {
      struct rtcm_message_s data;
      int file;
    } rtcm;

    /* BTstack Status */

    struct
    {
      struct btstack_status_s data;
      int file;
    } btstack;

  } pub;
};

static struct mapkit_data_s g_mapkit_data;

/**
 * RFCOMM can make use for ERTM. Due to the need to re-transmit packets,
 * a large buffer is needed to still get high throughput
 */
#ifdef ENABLE_L2CAP_ENHANCED_RETRANSMISSION_MODE_FOR_RFCOMM
static uint8_t ertm_buffer[4096];
static l2cap_ertm_config_t ertm_config =
{
    0,        /* ertm mandatory                       */
    8,        /* Number of retransmissions            */
    2000,     /* time before retransmission           */
    12000,    /* time after withc s-frames are sent   */
    1000,     /* l2cap ertm mtu                       */
    8,        /* Number of buffers for outgoing data  */
    8,        /* Number of buffers for incoming data  */
    0,        /* No FCS                               */
};

static int ertm_buffer_in_use;

/****************************************************************************
 * Name: rfcomm_ertm_request_handler
 ****************************************************************************/

static void rfcomm_ertm_request_handler(rfcomm_ertm_request_t * ertm_request)
{
    printf("ERTM Buffer requested, buffer in use %u\n", ertm_buffer_in_use);
    if (ertm_buffer_in_use) return;
    ertm_buffer_in_use = 1;
    ertm_request->ertm_config      = &ertm_config;
    ertm_request->ertm_buffer      = ertm_buffer;
    ertm_request->ertm_buffer_size = sizeof(ertm_buffer);
}

/****************************************************************************
 * Name: rfcomm_ertm_released_handler
 ****************************************************************************/

static void rfcomm_ertm_released_handler(uint16_t ertm_id)
{
    printf("ERTM Buffer released, buffer in use  %u, ertm_id %x\n", ertm_buffer_in_use, ertm_id);
    ertm_buffer_in_use = 0;
}
#endif

/****************************************************************************
 * Name: sensor_subscribe
 ****************************************************************************/

static struct pollfd *sensor_subscribe(struct mapkit_data_s *priv,
                                       FAR const char *topic_name,
                                       float topic_rate,
                                       int topic_latency)
{
  struct orb_object *obj = &priv->sub.gnss.obj;
  FAR struct pollfd *fds;
  size_t len;
  int fd;

  /* Allocate poll data */

  fds = malloc(sizeof(struct pollfd));
  if (!fds)
    {
      return NULL;
    }

  /* calculate the inverval */

  float interval = topic_rate ? (1000000 / topic_rate) : 0;

  /* get the object meta */

  len = strlen(topic_name) - 1;
  obj->instance = topic_name[len] - '0';
  obj->meta = orb_get_meta(topic_name);

  /* subscribe to the topic */

  fd = orb_subscribe_multi(obj->meta, obj->instance);
  if (fd < 0)
    {
      free(fds);
      return NULL;
    }

  /* set the interval */

  if (interval != 0)
    {
      orb_set_interval(fd, (unsigned)interval);

      if (topic_latency != 0)
        {
          orb_set_batch_interval(fd, topic_latency);
        }
    }

  fds->fd = fd;
  fds->events = POLLIN;

  /* Update the subsciber data */

  priv->sub.gnss.fds = fds;

  return fds;
}

/****************************************************************************
 * Name: spp_send_packet
 ****************************************************************************/

static void spp_send_packet(struct mapkit_data_s *priv)
{
  if (priv && priv->rfcomm_cid)
    {
      FAR uint8_t *buffer;
      size_t size;

      /* Send all saved data to the client */

      while (circbuf_used(&priv->circ))
        {
          /* Get the data chunk */

          buffer = circbuf_get_readptr(&priv->circ, &size);

          /* Send it to the client */

          rfcomm_send(priv->rfcomm_cid, buffer, size);

          /* Inform we read it */

          circbuf_readcommit(&priv->circ, size);
        }
    }
}

/****************************************************************************
 * Name: uorb_ondata
 *
 * Description:
 *   Print topic data by its print_message callback.
 *
 * Input Parameters:
 *   meta         The uORB metadata.
 *   fd           Subscriber handle.
 *
 * Returned Value:
 *   0 on success copy, otherwise -1
 ****************************************************************************/

static int uorb_ondata(struct mapkit_data_s *priv,
                       FAR const struct orb_metadata *meta,
                       int fd)
{
  char buffer[meta->o_size];
  FAR struct sensor_gnss_raw *gnss = (FAR struct sensor_gnss_raw *)buffer;
  int ret;

  ret = orb_copy(meta, fd, buffer);
  if (priv && ret == OK)
    {
      /* Save all recevied data */

      if (circbuf_write(&priv->circ, gnss->buf, gnss->len) != gnss->len)
        {
          /* Reset buffer */

          circbuf_reset(&priv->circ);
        }

      /* Skip the read data */

      orb_ioctl(fd, SNIOC_SKIP_BUFFER,
                (unsigned long)(uintptr_t)&meta->o_size);
    }

  return ret;
}

/****************************************************************************
 * Name: sensor_poll
 ****************************************************************************/

static void sensor_poll(struct mapkit_data_s *priv)
{
  /* Check the uORB topic event */

  if (priv && priv->sub.gnss.fds != NULL)
    {
      struct orb_object *obj = &priv->sub.gnss.obj;
      FAR struct pollfd *fds =  priv->sub.gnss.fds;

      while (poll(fds, NPOLLFDS, POLL_DELAY_SEC) == NPOLLFDS)
        {
          if (fds->revents & POLLIN)
            {
              if (uorb_ondata(priv, obj->meta, fds->fd) < 0)
                {
                  break;
                }
            }
        }
    }
}

/****************************************************************************
 * Name: sensor_timer_handler
 ****************************************************************************/

static void sensor_timer_handler(struct btstack_timer_source *ts)
{
  struct mapkit_data_s *priv = (struct mapkit_data_s *)ts->context;

  if (priv->rfcomm_cid)
    {
      /* Poll the sensor */

      sensor_poll(priv);

      /* request to send the data */

      rfcomm_request_can_send_now_event(priv->rfcomm_cid);
    }

  /* Check the publish topic validity */

  if (priv->pub.btstack.file > 0)
    {
      /* Update the connection state */

      priv->pub.btstack.data.state = (priv->pub.btstack.data.rfcid != 0);

      /* Publish the data */

      orb_publish(ORB_ID(btstack_status),
                  priv->pub.btstack.file,
                  &priv->pub.btstack.data);
    }

  /* Restart the timer */

  btstack_run_loop_set_timer(ts, POLL_PERIOD_MS);
  btstack_run_loop_add_timer(ts);
}

/****************************************************************************
 * Name: sensor_timer_setup
 ****************************************************************************/

static void sensor_timer_setup(void *context)
{
  struct mapkit_data_s *priv = (struct mapkit_data_s *)context;

  /* set one-shot timer */

  priv->poll_timer.process = &sensor_timer_handler;
  priv->poll_timer.context = context;
  btstack_run_loop_set_timer(&priv->poll_timer, POLL_PERIOD_MS);
  btstack_run_loop_add_timer(&priv->poll_timer);
}

/*
 * @section Packet Handler
 *
 * @text The packet handler of the combined example is just the combination of the individual packet handlers.
 */

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
  struct mapkit_data_s *priv = &g_mapkit_data;
  UNUSED(channel);

  bd_addr_t event_addr;
  uint8_t   rfcomm_channel_nr;

	switch (packet_type)
  {
		case HCI_EVENT_PACKET:
			switch (hci_event_packet_get_type(packet))
      {
        case HCI_EVENT_PIN_CODE_REQUEST:

            /* inform about pin code request */

            printf("Pin code request - using '0000'\n");
            hci_event_pin_code_request_get_bd_addr(packet, event_addr);
            gap_pin_code_response(event_addr, "0000");
            break;

        case HCI_EVENT_USER_CONFIRMATION_REQUEST:

            /* inform about user confirmation request */

            printf("SSP User Confirmation Request with numeric value '%06" PRIu32 "'\n", little_endian_read_32(packet, 8));
            printf("SSP User Confirmation Auto accept\n");
            break;

        case RFCOMM_EVENT_INCOMING_CONNECTION:
            rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
            rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
            priv->rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
            printf("RFCOMM channel 0x%02x requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
            rfcomm_accept_connection(priv->rfcomm_cid);
					break;

				case RFCOMM_EVENT_CHANNEL_OPENED:
					if (rfcomm_event_channel_opened_get_status(packet))
          {
            printf("RFCOMM channel open failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
          }
          else
          {
            priv->rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
            priv->rfcomm_mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
            printf("RFCOMM channel open succeeded. New RFCOMM Channel ID 0x%02x, max frame size %u\n", priv->rfcomm_cid,
                                                                                                       priv->rfcomm_mtu);

            /* disable page/inquiry scan to get max performance */

            gap_discoverable_control(0);
            gap_connectable_control(0);

            /* Update the publish topic data */

            priv->pub.btstack.data.rfcid = priv->rfcomm_cid;
            priv->pub.btstack.data.rfmtu = priv->rfcomm_mtu;
          }
					break;

        case RFCOMM_EVENT_CAN_SEND_NOW:
            spp_send_packet(priv);
            break;

        case RFCOMM_EVENT_CHANNEL_CLOSED:
            printf("RFCOMM channel closed\n");
            priv->rfcomm_cid = 0;

            /* re-enable page/inquiry scan again */

            gap_discoverable_control(1);
            gap_connectable_control(1);

            /* Update the publish topic data */

            priv->pub.btstack.data.rfcid = 0;
            priv->pub.btstack.data.rfmtu = 0;
            break;

        default:
            break;
			}
      break;

    case RFCOMM_DATA_PACKET:

      /* Check the publish topic validity */

      if (priv->pub.rtcm.file > 0)
        {
          /* Save the received data */

          memcpy(priv->pub.rtcm.data.msg,
                 packet,
                 priv->pub.rtcm.data.len = MIN(RTCM_MESSAGE_SIZE, size));

          /* Publish the data */

          orb_publish(ORB_ID(rtcm_message),
                      priv->pub.rtcm.file,
                      &priv->pub.rtcm.data);

          /* Send the RTCM data to the active GNSS device */

          orb_ioctl(priv->sub.gnss.fds->fd,
                    SNIOC_INJECT_DATA,
                    (unsigned long)(uintptr_t)&priv->pub.rtcm.data);
        }
        break;

    default:
        break;
	}
}

/****************************************************************************
 * Name: btstack_app
 ****************************************************************************/

int btstack_app(int argc, FAR char *argv[])
{
  struct mapkit_data_s *priv = &g_mapkit_data;
  int ret;

  (void)argc;
  (void)argv;

  /* Initialize the mapkit data */

  memset(priv, 0, sizeof(struct mapkit_data_s));

  /* Allocate the GNSS data buffer */

  ret = circbuf_init(&priv->circ, NULL, GNSS_MSG_SIZE * GNSS_MSG_NUM);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: circbuf_init error\n");
      return EXIT_FAILURE;
    }

  /* Register the event topic(s) */

  if (!sensor_subscribe(priv, "sensor_gnss_raw0", 1000/POLL_PERIOD_MS, 0))
    {
      circbuf_uninit(&priv->circ);
      return EXIT_FAILURE;
    }

  /* Create the publish file for RTCM */

  priv->pub.rtcm.file = orb_advertise(ORB_ID(rtcm_message), 0);
  priv->pub.rtcm.data.msg = g_rtcm_buf;
  priv->pub.rtcm.data.len = 0;

  /* Create the publish file for BTstack */

  priv->pub.btstack.file = orb_advertise(ORB_ID(btstack_status), 0);
  priv->pub.btstack.data.rfcid = 0;

  /* Configure the sensor poll timer */

  sensor_timer_setup((void*)priv);

  /* Initialize the BTstack */

  l2cap_init();

#ifdef ENABLE_BLE
  /* Initialize LE Security Manager. Needed for cross-transport key derivation */

  sm_init();
#endif

  /* reserved channel, mtu limited by l2cap */

  rfcomm_init();
  rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);

#ifdef ENABLE_L2CAP_ENHANCED_RETRANSMISSION_MODE_FOR_RFCOMM
  /* setup ERTM management */

  rfcomm_enable_l2cap_ertm(&rfcomm_ertm_request_handler, &rfcomm_ertm_released_handler);
#endif

  /* init SDP, create record for SPP and register with SDP */

  sdp_init();
  memset(priv->spp_serv_buf, 0, sizeof(priv->spp_serv_buf));
  spp_create_sdp_record(priv->spp_serv_buf,
                        sdp_create_service_record_handle(),
                        RFCOMM_SERVER_CHANNEL,
                        "SPP MapKit");
  btstack_assert(de_get_len(priv->spp_serv_buf) <= sizeof(priv->spp_serv_buf));
  sdp_register_service(priv->spp_serv_buf);

  /* register for HCI events */

  priv->hci_event.callback = &packet_handler;
  hci_add_event_handler(&priv->hci_event);

  /* short-cut to find other SPP Streamer */

  gap_set_class_of_device(TEST_COD);

  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
  gap_set_local_name("SPP Mapkit 00:00:00:00:00:00");
  gap_discoverable_control(1);

  /* turn on! */

  hci_power_control(HCI_POWER_ON);

  return 0;
}
