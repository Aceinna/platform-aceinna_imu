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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvomsgq.c,v $
$Author: aek $
$Revision: 3.20 $
$Date: 2008-04-27 14:45:15-07 $

Functions to create, wait and signal a message queue.

************************************************************/

#include <salvo.h>

#if OSENABLE_MESSAGE_QUEUES


/************************************************************
****                                                     ****
**                                                         **
MsgQ services when OSCOMBINE_EVENT_SERVICES is FALSE.

See event.c for fully documented version. The versions
herein should function identically, albeit without reduced
call graphs.

**                                                         **
****                                                     ****
************************************************************/
#if !OSCOMBINE_EVENT_SERVICES

/************************************************************
** OSCreateMsgQ(ecbP, mqcbP, msgPP, size)                  **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSCREATEMSGQ_MSGQ_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSCreateMsgQ( OStypeEcbP     ecbP,
                        OStypeMqcbP    mqcbP,
                        OStypeMsgQPP   msgPP,
                        OStypeMsgQSize msgQSize )
OSMONITOR_KEYWORD_POST
OSCREATEMSGQ_ATTR
{
  OStypeMqcbP lMqcbP;


  OSEnterCritical();
  OSIncCallDepth();

  ecbP->tcbP        = (OSgltypeTcbP) 0;
  ecbP->event.mqcbP = mqcbP;
  #if OSUSE_EVENT_TYPES
  ecbP->type        = OSEV_MSGQ;
  #endif

  lMqcbP            = ecbP->event.mqcbP;
  lMqcbP->count     = 0;
  lMqcbP->inPP      = msgPP;
  lMqcbP->outPP     = msgPP;
  lMqcbP->beginPP   = msgPP;
  lMqcbP->endPP     = &msgPP[msgQSize];

  OSDecCallDepth();
  OSLeaveCritical();

  return OSNOERR;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSCREATEMSGQ_MSGQ_C
#endif

/************************************************************
** OSWaitMsgQ(ecbP, msgPP, timeout)                        **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSWAITMSGQ_MSGQ_C
#include <salvomcg.h>
#endif

#if !OSENABLE_TIMEOUTS
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitMsgQ( OStypeEcbP  ecbP,
                      OStypeMsgPP msgPP )
OSMONITOR_KEYWORD_POST
#else
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitMsgQ( OStypeEcbP  ecbP,
                      OStypeMsgPP msgPP,
                      OStypeDelay timeout )
OSMONITOR_KEYWORD_POST
#endif
{
  union {
      OStypeErr     err;
      OStypeBoolean avail;
  } u;
  OStypeMqcbP       mqcbP;


  OSEnterCritical();
  OSIncCallDepth();


  #if OSENABLE_FAST_SIGNALING
  if (OScTcbP->status.bits.state == OSTCB_TASK_SIGNALED) {
    u.avail =  TRUE;
  }
  else
  #endif
  {
    #if OSENABLE_ERROR_CHECKING
    if (ecbP->event.mqcbP == 0) {
      u.avail = FALSE;
    }
    else
    #endif
    {
      u.avail = ecbP->event.mqcbP->count;
    }
  }


  #if !OSENABLE_TIMEOUTS
  u.err = OSWaitEvent(ecbP, u.avail);
  #else
  u.err = OSWaitEvent(ecbP, u.avail, timeout);
  #endif


  #if OSENABLE_FAST_SIGNALING
  // As with messages, a signaled task's message
  //  pointer is stored in the task's tcb.
  if (u.err & OSERR_SIGNALED) {
    *msgPP = OScTcbP->msgP;
    OScTcbP->msgP = (OStypeMsgP) 0;

    OSMsg("OSWaitMsgQ",
          OSMakeStr("task % d acquired message %d (now %d).",
                    OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS), OScTcbP->msgP));
  }
  else
  #endif

  // This is what we do if the task waited a message
  //  queue that had been previously signaled.
  if ((u.err & OSERR_AVAILABLE) != 0)
  {
    mqcbP = ecbP->event.mqcbP;

    *msgPP = *(mqcbP->outPP);
    if (++(mqcbP->outPP) == mqcbP->endPP) {
      mqcbP->outPP = mqcbP->beginPP;
    }
    mqcbP->count--;

    OSMsg("OSWaitMsgQ",
          OSMakeStr("task % d acquired message %d from queue.",
                    OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)));
  }


  // Here's what to do in case of a timeout.
  else if ((u.err & OSERR_TIMEOUT) != 0) {
    *msgPP = (OStypeMsgP) 0;
  }


  OSDecCallDepth();
  OSLeaveCritical();

  return (u.err);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSWAITMSGQ_MSGQ_C
#endif


/************************************************************
** OSSignalMsgQ(ecbP, msgP)                                **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSIGNALMSGQ_MSGQ_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSSignalMsgQ( OStypeEcbP ecbP,
                        OStypeMsgP msgP )
OSMONITOR_KEYWORD_POST
OSSIGNALMSGQ_ATTR
{
  union {
      OStypeErr  err;
      OStypeTcbP tcbP;
  } u;
  OStypeMqcbP mqcbP;


  OSEnterCritical();
  OSIncCallDepth();

  #if OSENABLE_BOUNDS_CHECKING
  if ((ecbP < OSECBP(1)) || (ecbP > OSECBP(OSEVENTS_LIMIT))) {
    OSWarn("OSSignalMsgQ",
           OSMakeStr("message queue %d nonexistent or invalid.",
                     OSeID(ecbP, OSEVENTS)));

    u.err = OSERR_BAD_P;
  }

  else
  #endif
  {
    #if OSENABLE_ERROR_CHECKING && OSUSE_EVENT_TYPES
    if (ecbP->type != OSEV_MSGQ) {
      OSWarn("OSSignalMsgQ",
             OSMakeStr("event %d is not a message queue.",
                       OSeID(ecbP, OSEVENTS)));
      u.err = OSERR_EVENT_BAD_TYPE;
    }

    else
    #endif
    {
      mqcbP = ecbP->event.mqcbP;

      #if OSENABLE_ERROR_CHECKING
      if (!mqcbP) {
        OSWarn("OSSignalMsgQ",
               OSMakeStr("message queue %d does not exist.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_CB_UNINIT;
      }

      else
      #endif

      #if !OSENABLE_FAST_SIGNALING
      if ((mqcbP->count) && (mqcbP->inPP == mqcbP->outPP)) {
        OSWarn("OSSignalMsgQ",
               OSMakeStr("message queue %d full.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_FULL;
      }

      else {
        *(mqcbP->inPP) = msgP;
        if (++(mqcbP->inPP) == mqcbP->endPP) {
          mqcbP->inPP = mqcbP->beginPP;
        }
        mqcbP->count++;

        u.tcbP = ecbP->tcbP;

        if (u.tcbP) {
          OSInsSigQ(u.tcbP, ecbP);
        }

        u.err = OSNOERR;
      }

      #else
      if (!ecbP->tcbP && mqcbP->count && (mqcbP->inPP == mqcbP->outPP)) {
        OSWarn("OSSignalMsgQ",
               OSMakeStr("message queue %d full.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_FULL;
      }

      else {
        u.tcbP = ecbP->tcbP;

        if (u.tcbP) {
          u.tcbP->status.bits.state = OSTCB_TASK_SIGNALED;
          OSInsSigQ(u.tcbP, ecbP);
          u.tcbP->msgP = msgP;
        }
        else {
          *(mqcbP->inPP) = msgP;
          if (++(mqcbP->inPP) == mqcbP->endPP) {
            mqcbP->inPP = mqcbP->beginPP;
          }
          mqcbP->count++;
        }

        u.err = OSNOERR;
      }
      #endif
    }
  }

  OSDecCallDepth();
  OSLeaveCritical();

  return(u.err);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSSIGNALMSGQ_MSGQ_C
#endif

#endif /* #if !OSCOMBINE_EVENT_SERVICES */

#endif /* #if OSENABLE_MESSAGE_QUEUES */



