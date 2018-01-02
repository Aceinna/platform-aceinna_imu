/************************************************************
The contents of this file are subject to the Pumpkin Salvo
License (the "License").  You may not use this file except
in compliance with the License. You may obtain a copy of
the License at http://www.pumpkininc.com, or from
warranty@pumpkininc.com.

Software distributed under the License is distributed on an
"AS IS" basis, WITHOUT WARRANTY OF ANY KIND, either express
or implied. See the License for specific language governing
the warranty and the rights and limitations under the
License.

The Original Code is Salvo - The RTOS that runs in tiny
places(TM). Copyright (C) 1995-2008 Pumpkin, Inc. and its
Licensor(s). All Rights Reserved.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvomsgq4.c,v $
$Author: aek $
$Revision: 3.9 $
$Date: 2008-04-27 14:45:34-07 $

Function to discern the number of elements in a message
queue. Created because because the pointer arithmetic
inside OSMsgQEmpty() is very expensive, cycle-wise, on targets
where the data pointer size is larger than the native register
size (e.g. PIC18 + PICC-18, OSMsgQEmoty() calls awdiv).

************************************************************/

#include <salvo.h>

#if OSENABLE_MESSAGE_QUEUES


/************************************************************
****                                                     ****
**                                                         **
OSMsgQCount(ecbP)

Returns: number of messages pointers already in the message
      queue.

**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSMSGQCOUNT_MSGQ4_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeMsgQSize OSMsgQCount ( OStypeEcbP ecbP )
OSMONITOR_KEYWORD_POST
OSMSGQCOUNT_ATTR
{
  OStypeMsgQSize  size;

  OSEnterCritical();
  OSIncCallDepth();

  size = ecbP->event.mqcbP->count;

  OSDecCallDepth();
  OSLeaveCritical();

  return size;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSMSGQCOUNT_MSGQ4_C
#endif

#endif /* #if OSENABLE_MESSAGE_QUEUES */



