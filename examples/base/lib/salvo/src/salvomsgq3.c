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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvomsgq3.c,v $
$Author: aek $
$Revision: 3.14 $
$Date: 2008-04-27 14:45:34-07 $

Functions to discern the number of elements in a message
queue.

************************************************************/

#include <salvo.h>

#if OSENABLE_MESSAGE_QUEUES

/************************************************************
****                                                     ****
**                                                         **
abs()

MPLAB-C18 has no abs(), so let's use this local one.


**                                                         **
****                                                     ****
************************************************************/
#if OSCOMPILER == OSMPLAB_C18
static int abs (int a )
{
    if ( a < 0 )
        return -a;
    return a;
}
#endif


/************************************************************
****                                                     ****
**                                                         **
OSMsgQEmpty(ecbP)

Determines whether a message queue is empty or not.

Returns: number of spaces available for message pointers in
          the message queue.

**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSMSGQEMPTY_MSGQ3_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeMsgQSize OSMsgQEmpty ( OStypeEcbP ecbP )
OSMONITOR_KEYWORD_POST
OSMSGQEMPTY_ATTR
{
  OStypeMsgQSize  size;
  OStypeMqcbP     mqcbP;

  OSEnterCritical();
  OSIncCallDepth();

  mqcbP = ecbP->event.mqcbP;
  size = (OStypeMsgQSize) (abs(mqcbP->endPP - mqcbP->beginPP) - mqcbP->count);

  OSDecCallDepth();
  OSLeaveCritical();

  return size;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSMSGQEMPTY_MSGQ3_C
#endif

#endif /* #if OSENABLE_MESSAGE_QUEUES */



