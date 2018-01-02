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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvomsg.c,v $
$Author: aek $
$Revision: 3.18 $
$Date: 2008-04-27 14:45:15-07 $

Functions to create, wait and signal a message.

************************************************************/

#include <salvo.h>

#if OSENABLE_MESSAGES


/************************************************************
****                                                     ****
**                                                         **
Msg services when OSCOMBINE_EVENT_SERVICES is FALSE.

See event.c for fully documented version. The versions
herein should function identically, albeit without reduced
call graphs.

**                                                         **
****                                                     ****
************************************************************/
#if !OSCOMBINE_EVENT_SERVICES

/************************************************************
** OSCreateMsg(ecbP, msgP)                                 **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSCREATEMSG_MSG_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSCreateMsg( OStypeEcbP ecbP,
                       OStypeMsgP msgP )
OSMONITOR_KEYWORD_POST
OSCREATEMSG_ATTR
{
  OSEnterCritical();
  OSIncCallDepth();

  ecbP->tcbP       = (OSgltypeTcbP) 0;
  ecbP->event.msgP = msgP;
  #if OSUSE_EVENT_TYPES
  ecbP->type       = OSEV_MSG;
  #endif


  OSDecCallDepth();
  OSLeaveCritical();

  return OSNOERR;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSCREATEMSG_MSG_C
#endif


/************************************************************
** OSWaitMsg(ecbP, msgPP, timeout)                         **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSWAITMSG_MSG_C
#include <salvomcg.h>
#endif

#if !OSENABLE_TIMEOUTS
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitMsg( OStypeEcbP  ecbP,
                     OStypeMsgPP msgPP )
OSMONITOR_KEYWORD_POST
#else
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitMsg( OStypeEcbP  ecbP,
                     OStypeMsgPP msgPP,
                     OStypeDelay timeout )
OSMONITOR_KEYWORD_POST
#endif
{
  union {
      OStypeErr     err;
      OStypeBoolean avail;
  } u;


  OSEnterCritical();
  OSIncCallDepth();


  #if OSENABLE_FAST_SIGNALING
  if (OScTcbP->status.bits.state == OSTCB_TASK_SIGNALED) {
    u.avail =  TRUE;
  }
  else
  #endif
  {
    u.avail = FALSE;
    if (ecbP->event.msgP != (OStypeMsgP) 0) {
      u.avail++;
    }
  }


  #if !OSENABLE_TIMEOUTS
  u.err = OSWaitEvent(ecbP, u.avail);
  #else
  u.err = OSWaitEvent(ecbP, u.avail, timeout);
  #endif

                              
  // When fast signaling, the message pointer was
  //  copied to the task's tcb msgP field when the
  //  message was signaled. Now, pass it "up" to the
  //  calling task. Note that not only must fast signaling
  //  be enabled, but the task must have been made
  //  by the signaling for this to take place.
  #if OSENABLE_FAST_SIGNALING
  if (u.err & OSERR_SIGNALED) {
    *msgPP = OScTcbP->msgP;

    #if OSCLEAR_UNUSED_POINTERS
    OScTcbP->msgP = (OStypeMsgP) 0;
    #endif

    OSMsg("OSWaitMsg",
          OSMakeStr("task % d acquired message %d (now %d).",
                    OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS), OScTcbP->msgP));
  }
  else
  #endif
      
                              
  // If the message had already been signaled before the
  //  task waited the message, then the message is stored
  //  in the ecb, and we need to get it from there. In
  //  this case, the message MUST be cleared, o/wise it
  //  would persist indefinitely.
  if ((u.err & OSERR_AVAILABLE) != 0) {
    *msgPP = ecbP->event.msgP;
    ecbP->event.msgP = (OStypeMsgP) 0;

    OSMsg("OSWaitMsg",
          OSMakeStr("task % d acquired message %d (now %d).",
                    OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS), ecbP->event.msgP));
  }


  // Lastly, the task may simply have timed out waiting
  //  for the event. In that case, the local msgP must
  //  be cleared.
  else if ((u.err & OSERR_TIMEOUT) != 0) {
    *msgPP = (OStypeMsgP) 0;
  }

  OSDecCallDepth();
  OSLeaveCritical();

  return (u.err);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSWAITMSG_MSG_C
#endif

/************************************************************
** OSSignalMsg(ecbP, msgP)                                 **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSIGNALMSG_MSG_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSSignalMsg( OStypeEcbP ecbP,
                       OStypeMsgP msgP )
OSMONITOR_KEYWORD_POST
OSSIGNALMSG_ATTR
{
  union {
      OStypeErr  err;
      OStypeTcbP tcbP;
  } u;


  OSEnterCritical();
  OSIncCallDepth();

  #if OSENABLE_BOUNDS_CHECKING
  if ((ecbP < OSECBP(1)) || (ecbP > OSECBP(OSEVENTS_LIMIT))) {
    OSWarn("OSSignalMsg",
           OSMakeStr("message %d nonexistent or invalid.",
                     OSeID(ecbP, OSEVENTS)));

    u.err = OSERR_BAD_P;
  }

  else
  #endif
  {
    #if OSENABLE_ERROR_CHECKING && OSUSE_EVENT_TYPES
    if (ecbP->type != OSEV_MSG)
    {
        OSWarn("OSSignalMsg",
          OSMakeStr("event %d is not a message.",
            OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_BAD_TYPE;
    }
    else
    #endif
    {
      #if !OSENABLE_FAST_SIGNALING
      // In the slow signaling case, we can't do
      //  anything until a task picks up the existing
      //  message.
      if (ecbP->event.msgP) {
        OSWarn("OSSignalMsg",
               OSMakeStr("message %d already present.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_FULL;
      }

      // The message pointer is stored in the ecb,
      //  the waiting task (if any) is made eligible
      //  so that it can get the message when it runs.
      else {
        ecbP->event.msgP = msgP;

        u.tcbP = ecbP->tcbP;

        if (u.tcbP) {
          OSInsSigQ(u.tcbP, ecbP);
        }

        u.err = OSNOERR;
      }

      #else
      // OSENABLE_FAST_SIGNALING case.
      // In the fast signaling case, a message is
      //  waiting only if there's no task waiting the
      //  message.
      if (!ecbP->tcbP && ecbP->event.msgP) {
        OSWarn("OSSignalMsg",
               OSMakeStr("message %d already present.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_FULL;
      }

      // O/wise either launch the waiting task, with
      //  the msgP in its tcb, or just copy the msgP
      //  to the ecb.
      else {
        u.tcbP = ecbP->tcbP;

        if (u.tcbP) {
          u.tcbP->status.bits.state = OSTCB_TASK_SIGNALED;
          OSInsSigQ(u.tcbP, ecbP);
          u.tcbP->msgP = msgP;
        }
        else {
            ecbP->event.msgP = msgP;
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
#undef __OSSIGNALMSG_MSG_C
#endif


#endif /* #if !OSCOMBINE_EVENT_SERVICES */

#endif /* #if OSENABLE_MESSAGES */



