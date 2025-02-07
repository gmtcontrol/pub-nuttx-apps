/*
*********************************************************************************************************
*                             	  GMT CONTROL QUEUE MANAGEMENT MODULE
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
*                                      	QUEUE MANAGEMENT MODULE
*
*                                               GENERIC
*
*
* Filename      : queue.c
* Version       : V1.00
* Programmer(s) : tmk
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "queue.h"
#include <string.h>


/*
*********************************************************************************************************
*                                             LOCAL MACROS
*********************************************************************************************************
*/

#ifdef USE_QUE_PROT
#define PROT_ENTER(x)   osMutexWait(x->Mutex, osWaitForever)
#define PROT_EXIT(x)		osMutexRelease(x->Mutex)
#define PROT_OK         osOK
#else
#define PROT_ENTER(x)   (0)
#define PROT_EXIT(x)
#define PROT_OK         (0)
#endif


/*******************************************************************************
* Function Name  : MakeEmpty
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void MakeEmpty(queue_t *psQueue)
{
	psQueue->Count = 0;
	psQueue->Front = 0;
	psQueue->Rear  = 0;
}//MakeEmpty


/*******************************************************************************
* Function Name  : Succ
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static int32_t Succ(queue_t *psQueue, int32_t Value)
{
	if (++Value == psQueue->Capacity)
	{
	Value = 0;
	}//if

	return Value;
}//Succ


/*******************************************************************************
* Function Name  : QIsFull
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t QIsFull(queue_t *psQueue)
{
	return (psQueue->Count == psQueue->Capacity);
}//QIsFull


/*******************************************************************************
* Function Name  : QIsEmpty
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t QIsEmpty(queue_t *psQueue)
{
	return (psQueue->Count == 0);
}//IsEmpty


/*******************************************************************************
* Function Name  : QGetCount
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t QGetCount(queue_t *psQueue)
{
	return psQueue->Count;
}//QGetCount


/*******************************************************************************
* Function Name  : QGetEmptyCount
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int32_t QGetEmptyCount(queue_t *psQueue)
{
	return (psQueue->Capacity - psQueue->Count);
}//QGetEmptyCount


/*******************************************************************************
* Function Name  : QCreate
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t QCreate(queue_t *psQueue, uint16_t dwCapacity, uint16_t dwItemSize, void *pvItems)
{
	MakeEmpty(psQueue);
	psQueue->Capacity = dwCapacity;
	psQueue->ItemSize = dwItemSize;
	psQueue->pvItems  = pvItems;
#ifdef USE_QUE_PROT
	psQueue->Mutex    = osMutexCreate(0);
#else
    psQueue->Mutex    = 0;
#endif
	memset(psQueue->pvItems, 0, (dwCapacity * dwItemSize));

	return 1;
}//QCreate


/*******************************************************************************
* Function Name  : QDestroy
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t QDestroy(queue_t *psQueue)
{
#ifdef USE_QUE_PROT
    if (psQueue->Mutex)
    {
        osMutexRelease(psQueue->Mutex);
        osMutexDelete(psQueue->Mutex);
    }//if
#endif
	memset(psQueue->pvItems, 0, (psQueue->Capacity * psQueue->ItemSize));
	memset(psQueue, 0, sizeof(queue_t));

	return 1;
}//QDestroy


/*******************************************************************************
* Function Name  : QFlush
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t QFlush(queue_t *psQueue)
{
	psQueue->Front = 0;
	psQueue->Rear  = 0;
	psQueue->Count = 0;

	return 1;
}//QFlush


/*******************************************************************************
* Function Name  : QReserve
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void * QReserve(queue_t *psQueue)
{
	void * buf = NULL;

	if (!QIsFull(psQueue))
	{
		if (PROT_ENTER(psQueue) != PROT_OK)
		    return NULL;

		buf = (void*)(((uint8_t*)psQueue->pvItems) + (psQueue->Rear * psQueue->ItemSize));
		psQueue->Reserved = 1;
	}//if

	return buf;
}//QReserve


/*******************************************************************************
* Function Name  : QRelease
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void QRelease(queue_t *psQueue)
{
	psQueue->Reserved = 0;
    PROT_EXIT(psQueue);
}//QRelease


/*******************************************************************************
* Function Name  : QUpdate
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t QUpdate(queue_t *psQueue)
{
	if (psQueue->Reserved)
	{
	    psQueue->Count++;
	    psQueue->Rear = Succ(psQueue, psQueue->Rear);
		PROT_EXIT(psQueue);

	return 1;
	}//if

	return 0;
}//QUpdate


/*******************************************************************************
* Function Name  : QEnqueue
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t QEnqueue(queue_t *psQueue, void *pvItem)
{
	if (!QIsFull(psQueue))
	{
		if (PROT_ENTER(psQueue) != PROT_OK)
		    return 0;

		memcpy((void*)(((uint8_t*)psQueue->pvItems) + (psQueue->Rear * psQueue->ItemSize)),
			    pvItem,
			    psQueue->ItemSize);

	    psQueue->Count++;
	    psQueue->Rear = Succ(psQueue, psQueue->Rear);
		PROT_EXIT(psQueue);

	return 1;
	}//if

	return 0;
}//QEnqueue


/*******************************************************************************
* Function Name  : QDequeue
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t QDequeue(queue_t *psQueue, void *pvItem)
{
	if (!QIsEmpty(psQueue))
	{
		if (PROT_ENTER(psQueue) != PROT_OK)
		    return 0;

		memcpy(pvItem,
			   (void*)((uint8_t*)(psQueue->pvItems) + (psQueue->Front * psQueue->ItemSize)),
			   psQueue->ItemSize);

	psQueue->Count--;
	psQueue->Front = Succ(psQueue, psQueue->Front);
		PROT_EXIT(psQueue);

	return 1;
	}//if

	return 0;
}//QDequeue