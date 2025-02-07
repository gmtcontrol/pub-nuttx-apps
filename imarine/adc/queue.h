/*
*********************************************************************************************************
*                             	 GMT CONTROL queue_t MANAGEMENT MODULE
*
*                            (c) Copyright 2014; Gmt, Inc.; Istanbul, TR
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                      	queue_t MANAGEMENT MODULE
*
*                                               GENERIC
*
*
* Filename      : queue.h
* Version       : V1.00
* Programmer(s) : tmk
*********************************************************************************************************
*/


#ifndef __QUEUE_H__
#define __QUEUE_H__


/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/

#include <stdint.h>


/*
*********************************************************************************************************
*                                              GLOBAL TYPES
*********************************************************************************************************
*/


/**
 * Queue structure.
 *
 * This structure is used to manage queue buffers.
 *
 * \hideinitializer
 */
typedef struct __queue_t
{
	int32_t  Front;
	int32_t  Rear;
	int32_t  Count;
	int32_t  Capacity;
	int32_t  ItemSize;
	int32_t	 Reserved;
	void 	  *pvItems;
	void 	  *Mutex;
} queue_t;

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

uint8_t  QCreate	   	  (queue_t *psQueue, uint16_t dwCapacity, uint16_t dwItemSize, void *pvItems);
uint8_t  QDestroy	      (queue_t *psQueue);
uint8_t  QEnqueue	      (queue_t *psQueue, void *pvItem);
uint8_t  QDequeue	      (queue_t *psQueue, void *pvItem);
uint8_t  QIsFull		    (queue_t *psQueue);
uint32_t QGetCount      (queue_t *psQueue);
uint8_t  QIsEmpty	      (queue_t *psQueue);
int32_t  QGetEmptyCount (queue_t *psQueue);
void 	  *QReserve	      (queue_t *psQueue);
void 	   QRelease	      (queue_t *psQueue);
uint8_t  QUpdate		    (queue_t *psQueue);
uint8_t  QFlush         (queue_t *psQueue);

#endif // __QUEUE_H__
