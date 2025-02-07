/****************************************************************************
 * apps/system/xmodem/xmodem.c
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

#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>

#include <nuttx/crc16.h>

#include "xmodem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_XMODEM_DEBUG_FILEPATH
#  define xmodem_debug(...) \
  do \
    { \
      dprintf(ctx->debug_fd, ##__VA_ARGS__); \
      fsync(ctx->debug_fd); \
    } \
  while(0)

#else
#  define xmodem_debug(...)
#endif

#define SOH           0x01  /* Start of 128-byte data packet */
#define STX           0x02  /* Start of 1024-byte data packet */
#define STC           0x03  /* Start of custom byte data packet */
#define EOT           0x04  /* End of transmission */
#define ACK           0x06  /* Acknowledge */
#define NAK           0x15  /* Negative acknowledge */
#define CAN           0x18  /* Two of these in succession aborts transfer */
#define CTRLZ         0x1A  /* Abort transfer */
#define ABORT1        0x41  /* 'A' == 0x41, abort by user */
#define CRC16         0x43  /* 'C' == 0x43, request 16-bit CRC */
#define ABORT2        0x61  /* 'a' == 0x61, abort by user */

#define MAXRETRY      16
#define MAXRETRANS    30

#define NAK_TIMEOUT   (1000)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmodem_recv_buffer
 ****************************************************************************/

static int xmodem_recv_buffer(FAR struct xmodem_ctx_s *ctx, FAR uint8_t *buf,
                              size_t size)
{
  size_t i = 0;

  xmodem_debug("recv buffer data, read size is %zu\n", size);
  while (i < size)
    {
      ssize_t ret = read(ctx->recvfd, buf + i, size - i);
      if (ret >= 0)
        {
          xmodem_debug("recv buffer data, size %zd\n", ret);
          i += ret;
        }
      else
        {
          xmodem_debug("recv buffer error, ret %d\n", -errno);
          usleep(NAK_TIMEOUT * 1000);
          return -errno;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: xmodem_send_buffer
 ****************************************************************************/

static int xmodem_send_buffer(FAR struct xmodem_ctx_s *ctx,
                              FAR const uint8_t *buf, size_t size)
{
  size_t i = 0;

  xmodem_debug("send buffer data, write size is %zu\n", size);
  while (i < size)
    {
      ssize_t ret = write(ctx->sendfd, buf, size);
      if (ret >= 0)
        {
          xmodem_debug("send buffer data, size %zd\n", ret);
          i += ret;
        }
      else
        {
          xmodem_debug("send buffer error, ret %d\n", -errno);
          return -errno;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: _outbyte
 ****************************************************************************/

static int _outbyte(FAR struct xmodem_ctx_s *ctx, uint8_t byte)
{
  return xmodem_send_buffer(ctx, &byte, 1);
}

/****************************************************************************
 * Name: _inbyte
 ****************************************************************************/

static int _inbyte(FAR struct xmodem_ctx_s *ctx, uint32_t _dly)
{
  uint8_t byte;
  int ret;

  if ((ret = xmodem_recv_buffer(ctx, &byte, 1)) == 0)
  {
    return (int)byte;
  }
  else
  {
    return -1;
  }
}

/****************************************************************************
 * Name: _inflush
 ****************************************************************************/

static void _inflush(FAR struct xmodem_ctx_s *ctx)
{
  tcflush(ctx->recvfd, TCIFLUSH);
}

/****************************************************************************
 * Name: check
 ****************************************************************************/

static int check(int enab, const uint8_t *buf, int sz)
{
	if (enab)
  {
		uint16_t crc  = crc16(buf, sz);
		uint16_t tcrc = (buf[sz]<<8)+buf[sz+1];
		if (crc == tcrc) {
			return 1;
    }
	}
	else
  {
		int i;
		uint8_t cks = 0;
		for (i = 0; i < sz; ++i) {
			cks += buf[i];
		}
		if (cks == buf[sz]) {
		  return 1;
    }
	}

	return 0;
}

/****************************************************************************
 * Name: xmodem_recv_file
 ****************************************************************************/

int xmodem_recv_file(FAR struct xmodem_ctx_s *ctx)
{
  struct xmodem_priv_s *priv = (struct xmodem_priv_s *)ctx->priv;
	uint8_t trychar = 'C';
	int bufsz, crc = 0;
	int i, c, len = 0;
	uint8_t *p;

  /* initial conditions */

  ctx->retrans  = MAXRETRANS;
	ctx->packet_no = 1;

	while (1)
  {
		for (ctx->retry = 0; ctx->retry < MAXRETRY; ++ctx->retry)
    {
			if (trychar)
      {
        _outbyte(ctx, trychar);
      }

			if ((c = _inbyte(ctx, NAK_TIMEOUT)) > 0)
      {
				switch (c)
        {
          case SOH:
            bufsz = XMODEM_PACKET_SIZE;
            goto start_recv;

          case STX:
            bufsz = XMODEM_PACKET_1K_SIZE;
            goto start_recv;

          case EOT:
            _inflush(ctx);
            _outbyte(ctx, ACK);

            /* normal end */
            return len;

          case CAN:
            if ((c = _inbyte(ctx, NAK_TIMEOUT)) == CAN)
            {
              _inflush(ctx);
              _outbyte(ctx, ACK);

              /* canceled by remote */
              printf("\r\nERROR: canceled by remote!\n");
              return -3;
            }
            break;

					case ABORT1:
					case ABORT2:
						_outbyte(ctx, CAN);
						_outbyte(ctx, CAN);

            /* aborted by remote */
            printf("\r\nERROR: aborted by remote!\n");
						return (-3);

          default:
            break;
				}
			}
		}

		if (trychar == 'C')
    {
      trychar = NAK;
      continue;
    }

		_inflush(ctx);
		_outbyte(ctx, CAN);
		_outbyte(ctx, CAN);
		_outbyte(ctx, CAN);

    /* sync error */
    printf("\r\nERROR: sync!\n");
		return -4;

	start_recv:
		if (trychar == 'C')
    {
      crc = 1;
    }

		trychar = 0;
		p = ctx->header;
		*p++ = c;
		for (i = 0;  i < (bufsz + (crc ? 1 : 0) + 3); ++i)
    {
			if ((c = _inbyte(ctx, NAK_TIMEOUT)) <= 0)
      {
        goto reject;
      }
			*p++ = c;
		}

		if (ctx->header[1] == (uint8_t)(~ctx->header[2]) &&
			 (ctx->header[1] == ctx->packet_no || ctx->header[1] == (uint8_t)ctx->packet_no-1) &&
			  check(crc, &ctx->header[XMODEM_PACKET_HEADER], bufsz))
    {
			if (ctx->header[1] == ctx->packet_no)
      {
        if ((bufsz > 0) && (priv->fd > 0))
				{
					len += bufsz;

          /* write the packet */

          if (write(priv->fd, &ctx->header[XMODEM_PACKET_HEADER], bufsz) < 0)
          {
            /* data write error */

            printf("\r\nERROR: data write!\n");
            return -6;
          }
				}

				++ctx->packet_no;
				ctx->retrans = MAXRETRANS + 1;
			}

			if (--ctx->retrans <= 0)
      {
				_inflush(ctx);
				_outbyte(ctx, CAN);
				_outbyte(ctx, CAN);
				_outbyte(ctx, CAN);

        /* too many retry error */

        printf("\r\nERROR: too many retry!\n");
				return -5;
			}
			_outbyte(ctx, ACK);
			continue;
		}

	reject:
		_inflush(ctx);
		_outbyte(ctx, NAK);
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmodem_recv
 ****************************************************************************/

int xmodem_recv(FAR struct xmodem_ctx_s *ctx)
{
  struct termios saveterm;
  struct termios term;
  int ret;

  if (ctx == NULL)
    {
      return -EINVAL;
    }

  /* allocate the packet buffer */

  ctx->header = calloc(1, XMODEM_PACKET_1K_SIZE + XMODEM_PACKET_OVERHEAD);
  if (ctx->header == NULL)
    {
      return -ENOMEM;
    }

#ifdef CONFIG_SYSTEM_XMODEM_DEBUG_FILEPATH
  ctx->debug_fd = open(CONFIG_SYSTEM_XMODEM_DEBUG_FILEPATH,
                       O_CREAT | O_TRUNC | O_WRONLY, 0666);
  if (ctx->debug_fd < 0)
    {
      free(ctx->header);
      return -errno;
    }
#endif

  tcgetattr(ctx->recvfd, &term);
  memcpy(&saveterm, &term, sizeof(struct termios));
  cfmakeraw(&term);
  term.c_cc[VTIME] = 15;
  term.c_cc[VMIN] = 255;
  tcsetattr(ctx->recvfd, TCSANOW, &term);

  ret = xmodem_recv_file(ctx);

  tcsetattr(ctx->recvfd, TCSANOW, &saveterm);
#ifdef CONFIG_SYSTEM_XMODEM_DEBUG_FILEPATH
  close(ctx->debug_fd);
#endif

  free(ctx->header);
  return ret;
}

/****************************************************************************
 * Name: xmodem_send
 ****************************************************************************/

int xmodem_send(FAR struct xmodem_ctx_s *ctx)
{
  return 0;
}