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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoevent.c,v $
$Author: aek $
$Revision: 3.25 $
$Date: 2008-04-27 14:45:14-07 $

Functions to create, wait and signal events.

See binsem.c for detailed descriptions of what we're
doing here.

************************************************************/

#include <salvo.h>

#if OSENABLE_EVENTS

#if OSCOMBINE_EVENT_SERVICES

/************************************************************
****                                                     ****
**                                                         **
Event services for OSCOMBINE_EVENT_SERVICES is TRUE

In this case, all event services are macros that call
OSCreateEvent(), OSWaitSignalEvent() and OSWaitEvent().
Variable argument lists are used to support all the
different event types.

Event services are combined to remove redundant, repeated
code when more than one event type is used. The entire
service is as "flat" as possible in order to reduce call
stack usage.


**                                                         **
****                                                     ****
************************************************************/


/************************************************************
****                                                     ****
**                                                         **
OSCreateEvent(ecbP, eType, ...)

Create an event and assign it its initial value. Variable
argument list is used to support the different event types
and the info required to initialize them.

Returns: OSNOERR

**                                                         **
****                                                     ****
************************************************************/
#define __OSCREATEEVENT_EVENT_C
#include <salvomcg.h>

OSMONITOR_KEYWORD_PRE
OStypeErr OSCreateEvent( OStypeEcbP  ecbP,
                         OStypeEType eType,
                         ... )
OSMONITOR_KEYWORD_POST
OSCREATEEVENT_ATTR
{
  va_list arg;

  #if OSENABLE_MESSAGE_QUEUES
  OStypeMsgPP    msgPP;
  OStypeMqcbP    mqcbP;
  OStypeMsgQSize size;
  #endif


  OSEnterCritical();
  OSIncCallDepth();

  // Initially no task is waiting the event.
  ecbP->tcbP = (OSgltypeTcbP) 0;

  // Set event type if enabled.
  #if OSUSE_EVENT_TYPES
  ecbP->type = eType;
  #endif

  // Here's the start of the event-specific ecb
  //  initialization. Event-specific arguments are
  //  extracted via va_arg(). If a particular event type
  //  is not being used, its corresponding code can be
  //  suppressed.
  va_start(arg, eType);

  switch (eType) {

    // BINSEM -- set binSem's initial value. Restrict
    //  its value to [0,1].
    #if OSENABLE_BINARY_SEMAPHORES
    case OSEV_BINSEM:
      ecbP->event.binSem = va_arg(arg, OStypeBinSem) & 0x01;
      break;
    #endif

    // EFLAG -- setup link to event flag's control
    //  block and set eFlag's initial value in control
    //  block.
    #if OSENABLE_EVENT_FLAGS
    case OSEV_EFLAG:
      ecbP->event.efcbP        = va_arg(arg, OStypeEfcbP);
      ecbP->event.efcbP->eFlag = va_arg(arg, OStypeEFlag);
      break;
    #endif

    // MSG -- set msgP's initial value.
    #if OSENABLE_MESSAGES
    case OSEV_MSG:
      ecbP->event.msgP = va_arg(arg, OStypeMsgP);
      break;
    #endif

    // MSGQ -- using local handle, setup the counter
    //  and pointers in the mqcb to reflect an
    //  initially empty msgQ.
    #if OSENABLE_MESSAGE_QUEUES
    case OSEV_MSGQ:
      ecbP->event.mqcbP = va_arg(arg, OStypeMqcbP);
      msgPP             = va_arg(arg, OStypeMsgPP);
      size              = va_arg(arg, OStypeMsgQSize);
      mqcbP             = ecbP->event.mqcbP;
      mqcbP->count      = 0;
      mqcbP->inPP       = msgPP;
      mqcbP->outPP      = msgPP;
      mqcbP->beginPP    = msgPP;
      mqcbP->endPP      = &msgPP[size];
      break;
    #endif

    // SEM -- set sem's intial value.
    #if OSENABLE_SEMAPHORES
    case OSEV_SEM:
      ecbP->event.sem = va_arg(arg, OStypeSem);
      break;
    #endif

    default:
        break;

  } /* switch() */

  /* Now we're done with event-specific initialization.    */
  va_end(arg);

  OSDecCallDepth();
  OSLeaveCritical();

  OSMsgRtn("OSCreateEvent",
           OSMakeStr("event %d of type %d created.", OSeID(ecbP),
                     eType), 
           (OStypeErr) OSNOERR);
}

#include <salvoscg.h>
#undef __OSCREATEEVENT_EVENT_C


/************************************************************
****                                                     ****
**                                                         **
OSSignalEvent(ecbP, eType, ...)

If a task is waiting for the event specified, place the
event (already signaled by the calling function) into the
event queue.

Returns:    OSERR_BAD_P if a bad ecbP is passed.
            OSERR_EVENT_BAD_TYPE if the ecb pointed to is
             not of the correct type.
            OSERR_EVENT_FULL if the binSem, msg, msgQ or
             sem events are already non-zero, non-zero,
             full or at their maximum value, respectively.
            OSERR_EVENT_CB_UNINIT if an efcb or mqcb does
             not exist for the specified event.
            OSNOERR if the event is successfully signaled.

**                                                         **
****                                                     ****
************************************************************/
#define __OSSIGNALEVENT_EVENT_C
#include <salvomcg.h>

OSMONITOR_KEYWORD_PRE
OStypeErr OSSignalEvent( OStypeEcbP  ecbP,
                         OStypeEType eType,
                         ... )
OSMONITOR_KEYWORD_POST
OSSIGNALEVENT_ATTR
{
  va_list arg;

  union {
      #if OSENABLE_EVENT_FLAGS
      OStypeEFlag eFlag;
      #endif
      OStypeErr   err;
      OStypeTcbP  tcbP;
  } u;

  #if OSENABLE_EVENT_FLAGS || OSENABLE_MESSAGE_QUEUES
  union {
      #if OSENABLE_EVENT_FLAGS
      OStypeEfcbP efcbP;
      #endif

      #if OSENABLE_MESSAGE_QUEUES
      OStypeMqcbP mqcbP;
      #endif
  } u2;
  #endif

  #if OSENABLE_EVENT_FLAGS
  OStypeOption options;
  OStypeEFlag mask;
  #endif


  OSEnterCritical();
  OSIncCallDepth();

  #if OSENABLE_BOUNDS_CHECKING
  // Punt if ecbP is clearly bad.
  if ((ecbP < OSECBP(1)) || (ecbP > OSECBP(OSEVENTS_LIMIT))) {
    OSWarn("OSSignalEvent",
           OSMakeStr("event %d of type %d nonexistent or invalid.",
                     OSeID(ecbP), eType));

    u.err = OSERR_BAD_P;
  }

  else
  #endif
  {
    #if OSENABLE_ERROR_CHECKING && OSUSE_EVENT_TYPES
    // Ensure we're signaling the correct event type.
    if (ecbP->type != eType) {
      OSWarn("OSSignalEvent",
             OSMakeStr("event %d is not of type %d.",
                       OSeID(ecbP), eType));

      u.err = OSERR_EVENT_BAD_TYPE;
    }

    else
    #endif

    #if !OSENABLE_FAST_SIGNALING
    // Event-specific error checking ... if all is well
    //  when we're done error checking u.err will be
    //  FALSE and we'll be able to signal the event.
    {

      // Must set "default value" now.
      u.err = OSNOERR;

      switch (eType) {

        #if OSENABLE_BINARY_SEMAPHORES
        // BINSEM -- make sure binSem isn't already
        //  set ... since we only set the binSem
        //  (below), it's OK to check for non-zero
        //  instead of exact value of 1.
        case OSEV_BINSEM:
          if (ecbP->event.binSem) {
            OSWarn("OSSignalEvent",
                   OSMakeStr("binary semaphore %d already set.",
                             OSeID(ecbP, OSEVENTS)));

            u.err = OSERR_EVENT_FULL;
          }
          break;
        #endif

        #if OSENABLE_SEMAPHORES
        // SEM -- make sure semaphore doesn't wrap.
        case OSEV_SEM:
          if (ecbP->event.sem >= MAX_SEM) {
            OSWarn("OSSignalEvent",
                   OSMakeStr("semaphore %d maxed out.",
                             OSeID(ecbP, OSEVENTS)));

            u.err = OSERR_EVENT_FULL;
          }
          break;
        #endif

        #if OSENABLE_MESSAGES
        // MSG -- if there's already a message in
        //   there, don't overwrite it.
        case OSEV_MSG:
          if (ecbP->event.msgP) {
            OSWarn("OSSignalEvent",
                   OSMakeStr("message %d already present.",
                             OSeID(ecbP, OSEVENTS)));

            u.err = OSERR_EVENT_FULL;
          }
          break;
        #endif

        #if OSENABLE_EVENT_FLAGS
        // EFLAG -- using a local handle to speed
        //  things up, avoid null pointer for event
        //  flag that hasn't yet been created.
        case OSEV_EFLAG:
          u2.efcbP = ecbP->event.efcbP;

          #if OSENABLE_ERROR_CHECKING
          if (!u2.efcbP) {
            OSWarn("OSSignalEvent",
                   OSMakeStr("event flag %d does not exist.",
                             OSeID(ecbP, OSEVENTS)));
            u.err = OSERR_EVENT_CB_UNINIT;
          }
          #endif
          break;
        #endif

        #if OSENABLE_MESSAGE_QUEUES
        // MSGQ -- using a local handle to speed
        //  things up, avoid a null pointer for a
        //  msgQ that hasn't yet been created, and
        //   also check if msgQ is already full.
        case OSEV_MSGQ:
            u2.mqcbP = ecbP->event.mqcbP;

            #if OSENABLE_ERROR_CHECKING
            if (!u2.mqcbP) {
              OSWarn("OSSignalEvent",
                     OSMakeStr("message queue %d does not exist.",
                               OSeID(ecbP, OSEVENTS)));
              u.err = OSERR_EVENT_CB_UNINIT;
            }
            #endif

            if ((u2.mqcbP->count)
              && (u2.mqcbP->inPP == u2.mqcbP->outPP)) {
              OSWarn("OSSignalEvent",
                     OSMakeStr("message queue %d full.",
                               OSeID(ecbP, OSEVENTS)));
              u.err = OSERR_EVENT_FULL;
            }
            break;
        #endif

        default:
            break;

      } /* switch() */

      // Everything looks OK, so operate on the event
      //  and take care of any waiting tasks.
      if (!u.err) {

        va_start(arg, eType);

        switch (eType) {

          // event-specific processing,
          //  independent of whether or not a
          //  task is waiting the event.

          #if OSENABLE_BINARY_SEMAPHORES
          // BINSEM -- simple -- set binsem to 1.
          case OSEV_BINSEM:
              ecbP->event.binSem = 1;
              break;
          #endif

          #if OSENABLE_MESSAGES
          // MSG -- copy the message pointer to
          //  the ecb.
          case OSEV_MSG:
              ecbP->event.msgP = va_arg(arg, OStypeMsgP);
              break;
          #endif

          #if OSENABLE_SEMAPHORES
          // SEM -- simple, just increment it.
          case OSEV_SEM:
              (ecbP->event.sem)++;
              break;
          #endif

          #if OSENABLE_EVENT_FLAGS
          // EFLAG -- we'll only take further
          //  action if the bits change as a
          //  result of the operation.
          case OSEV_EFLAG:
            mask    = va_arg(arg, OStypeEFlag);
            options = va_arg(arg, OStypeOption);

            // Check if applying the mask will
            //  change the event flag. If not,
            //  there's no reason to signal
            //  anything. Using & is faster
            //  than doing compare. Also, clrs
            //  do not cause events.
            if (options & OSSET_EFLAG) {
              u.eFlag = (u2.efcbP->eFlag | mask);
            }
            else {
              u.eFlag = (u2.efcbP->eFlag & ~mask);
            }

            if (u.eFlag == u2.efcbP->eFlag) {
              OSWarn("OSSignalEvent",
                     OSMakeStr("no change to event flag %d.",
                               OSeID(ecbP, OSEVENTS)));
              u.err = OSERR_EVENT_FULL;
              goto OSSignalEventSkipSignal;
            }

            else {
              u2.efcbP->eFlag = u.eFlag;
            }

            if (!(options & OSSET_EFLAG)) {
              u.err = OSNOERR;
              goto OSSignalEventSkipSignal; //aek correct here?
            }

            break;
          #endif

          #if OSENABLE_MESSAGE_QUEUES
          // MSGQ -- copy the message pointer to
          //  the inP of the mqcb. Advance the
          //  inP and wrap if required. Increment
          //  the message count.
          case OSEV_MSGQ:
            *(u2.mqcbP->inPP)  = va_arg(arg, OStypeMsgP);
            if ( ++(u2.mqcbP->inPP) == u2.mqcbP->endPP )
                u2.mqcbP->inPP = u2.mqcbP->beginPP;
            u2.mqcbP->count++;
            break;
          #endif

          default:
            break;

        } /* switch() */


        // Now we're done processing event-specific
        //  arguments.
        va_end(arg);

        // event-independent processing. If a task
        //  is waiting this event, insert the task
        //  into the signaled queue. In all cases
        //  except event flags, just the highest-
        //  priority waiting task is put into the
        //  signaled queue. But when event flags
        //  are in use, when an event flag is
        //  signaled with waiting tasks, all the
        //  tasks must go into the signaled queue.
        u.tcbP = ecbP->tcbP;

        #if !OSENABLE_EVENT_FLAGS
        if (u.tcbP) {
        #else
        while (u.tcbP) {
        #endif

          // Put into signaled queue at inP.
          OSInsSigQ(u.tcbP, ecbP);

          // Event flags require all waiting
          //  tasks to be enqueued. if we're doing
          //  an event flag we need to loop,
          //  o/wise we're done.
          #if OSENABLE_EVENT_FLAGS
          if (eType == OSEV_EFLAG) {
            u.tcbP = ecbP->tcbP;
          }
          else {
            break;
          }
          #endif
        }

        // Done -- waiting task was inserted into
        //  the signaled queue.
        u.err = OSNOERR;

#if OSENABLE_EVENT_FLAGS
OSSignalEventSkipSignal:
      ;
#endif
    }
  } /* else */

  #else /* #if OSENABLE_FAST_SIGNALING */
  // Fast signaling code, Here we check to see if a
  //  task is waiting, and if so, make the task
  //  eligible. If not, act on the event type.
  {

    u.err = OSNOERR;

    va_start(arg, eType);

    switch (eType) {

      #if OSENABLE_BINARY_SEMAPHORES
      case OSEV_BINSEM:
        if (!ecbP->tcbP && ecbP->event.binSem) {
          OSWarn("OSSignalEvent",
                 OSMakeStr("binary semaphore %d already set.",
                           OSeID(ecbP, OSEVENTS)));

          u.err = OSERR_EVENT_FULL;
        }
        break;
      #endif

      #if OSENABLE_SEMAPHORES
      case OSEV_SEM:
        if (!ecbP->tcbP && (ecbP->event.sem >= MAX_SEM)) {
          OSWarn("OSSignalEvent",
                 OSMakeStr("semaphore %d maxed out.",
                           OSeID(ecbP)));

          u.err = OSERR_EVENT_FULL;
        }
        break;
      #endif

      #if OSENABLE_MESSAGES
        case OSEV_MSG:
          if (!ecbP->tcbP && ecbP->event.msgP) {
            OSWarn("OSSignalEvent",
                   OSMakeStr("message %d already present.",
                             OSeID(ecbP, OSEVENTS)));

            u.err = OSERR_EVENT_FULL;
          }
          break;
        #endif

      // Event flags are different because they
      //  need to be processed in order to find
      //  out whether any waiting tasks should
      //  be launched.
      #if OSENABLE_EVENT_FLAGS
      case OSEV_EFLAG:
        u2.efcbP = ecbP->event.efcbP;

        #if OSENABLE_ERROR_CHECKING
        if (!u2.efcbP) {
	        OSWarn("OSSignalEvent",
	               OSMakeStr("event flag %d does not exist.",
	                         OSeID(ecbP, OSEVENTS)));
          u.err = OSERR_EVENT_CB_UNINIT;
        }
        #endif

        mask    = va_arg(arg, OStypeEFlag);
        options = va_arg(arg, OStypeOption);

        if (options & OSSET_EFLAG) {
          u.eFlag = (u2.efcbP->eFlag | mask);
        }
        else {
            u.eFlag = (u2.efcbP->eFlag & ~mask);
        }

        if (u.eFlag == u2.efcbP->eFlag) {
          OSWarn("OSSignalEvent",
                 OSMakeStr("no change to event flag %d.",
                           OSeID(ecbP, OSEVENTS)));
          u.err = OSERR_EVENT_FULL;
        }
        else {
          u2.efcbP->eFlag = u.eFlag;
        }

        if (!(options & OSSET_EFLAG)) {
          u.err = OSNOERR;
        }

        break;
      #endif


      #if OSENABLE_MESSAGE_QUEUES
      case OSEV_MSGQ:
        u2.mqcbP = ecbP->event.mqcbP;

        #if OSENABLE_ERROR_CHECKING
        if (!u2.mqcbP) {
          OSWarn("OSSignalEvent",
                 OSMakeStr("message queue %d does not exist.",
                           OSeID(ecbP, OSEVENTS)));
          u.err = OSERR_EVENT_CB_UNINIT;
        }
        #endif

        if (!ecbP->tcbP && u2.mqcbP->count
          && (u2.mqcbP->inPP == u2.mqcbP->outPP)) {
          OSWarn("OSSignalEvent",
                 OSMakeStr("message queue %d full.",
                           OSeID(ecbP, OSEVENTS)));
          u.err = OSERR_EVENT_FULL;
        }
        break;
      #endif

      default:
        break;

    } /* switch() */

    // Everything looks OK, so operate on the event
    //  and take care of any waiting tasks.
    if (!u.err) {

      u.tcbP = ecbP->tcbP;

OSSignalEventLoop:
      if (u.tcbP) {

        OSInsSigQ(u.tcbP, ecbP);

        // Event flags require all waiting
        //  tasks be processed. Using the label
        //  is an easy way to implement a
        //  while()-like structure.
        #if OSENABLE_EVENT_FLAGS
        if (eType == OSEV_EFLAG) {
          u.tcbP = ecbP->tcbP;
          goto OSSignalEventLoop;
        }
        #endif
      }
      
      else {

        switch ( eType )
        {

          #if OSENABLE_BINARY_SEMAPHORES
          case OSEV_BINSEM:
            ecbP->event.binSem = 1;
            break;
          #endif

          #if OSENABLE_MESSAGES
          case OSEV_MSG:
            ecbP->event.msgP = va_arg(arg, OStypeMsgP);
            break;
          #endif

          #if OSENABLE_SEMAPHORES
          case OSEV_SEM:
            (ecbP->event.sem)++;
            break;
          #endif

          #if OSENABLE_MESSAGE_QUEUES
          case OSEV_MSGQ:
            *(u2.mqcbP->inPP)  = va_arg(arg, OStypeMsgP);
            if ( ++(u2.mqcbP->inPP) == u2.mqcbP->endPP )
                u2.mqcbP->inPP = u2.mqcbP->beginPP;
            u2.mqcbP->count++;
            break;
          #endif

          // Note that this will catch eFlags
          //  if enabled.
          default:
              break;

        } /* switch() */
      }

      // Now we're done processing event-specific
      //  arguments.
      va_end(arg);

      // Done -- waiting task was inserted into
      //  the signaled queue.
      u.err = OSNOERR;
      }

    } /* else */
    #endif /* #if !OSENABLE_FAST_SIGNALING */
  }

  // Clean up and return.
  OSDecCallDepth();
  OSLeaveCritical();

  return(u.err);
}

#include <salvoscg.h>
#undef __OSSIGNALEVENT_EVENT_C


/************************************************************
****                                                     ****
**                                                         **
OSWaitEvent(ecbP, eType, timeout, ...)

Make current task wait event. See non-combined version of
OSWaitEvent() (below) for comments. Only differences are
commented here.

NOTE: timeouts of 0 are ignored.

Returns:    Error codes are internal to Salvo and are not
            used at the user level. Error codes are read
            by OS_WaitXyz() and OSWaitXyz().

**                                                         **
****                                                     ****
************************************************************/
#define __OSWAITEVENT_EVENT_C
#include <salvomcg.h>

#if !OSENABLE_TIMEOUTS
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitEvent(OStypeEcbP  ecbP,
                      OStypeEType eType,
                      ... )
OSMONITOR_KEYWORD_POST
#else
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitEvent(OStypeEcbP  ecbP,
                      OStypeEType eType,
                      OStypeDelay timeout,
                      ... )
OSMONITOR_KEYWORD_POST
#endif
{
  va_list arg;
  union {
      OStypeErr     err;
      OStypeBoolean avail;
  } u;

  #if OSENABLE_EVENT_FLAGS
  OStypeEFlag  mask;
  OStypeOption options;
  #endif

  #if OSENABLE_EVENT_FLAGS || OSENABLE_MESSAGE_QUEUES
  union {
      #if OSENABLE_EVENT_FLAGS
      OStypeEFlag eFlag;
      #endif

      #if OSENABLE_MESSAGE_QUEUES
      OStypeMqcbP mqcbP;
      #endif
  } u2;
  #endif

  #if OSENABLE_MESSAGES || OSENABLE_MESSAGE_QUEUES
  OStypeMsgPP msgPP;
  #endif


  OSEnterCritical();
  OSIncCallDepth();

  #if !OSENABLE_TIMEOUTS
  va_start(arg, eType);
  #else
  va_start(arg, timeout);
  #endif

  #if OSENABLE_MESSAGES || OSENABLE_MESSAGE_QUEUES
  msgPP = va_arg(arg, OStypeMsgPP);
  #endif


  // Action is dependent on what state the task is in
  //  when it enters OSWaitEvent().
  #if OSENABLE_TIMEOUTS
  if (OScTcbP->status.bits.state == OSTCB_TASK_TIMED_OUT) {
    OScTcbP->status.bits.state   = OSTCB_TASK_ELIGIBLE;

    OScTcbP->status.bits.yielded = FALSE;

    // If we got here because we timed out, we need to
    //  wipe out msgP to indicate that that it's
    //  invalid.
    #if OSENABLE_MESSAGES || OSENABLE_MESSAGE_QUEUES
    #if OSENABLE_MESSAGES && !OSENABLE_MESSAGE_QUEUES
    if (eType == OSEV_MSG) {
    #elif !OSENABLE_MESSAGES && OSENABLE_MESSAGE_QUEUES
    if (eType == OSEV_MSGQ) {
    #elif OSENABLE_MESSAGES && OSENABLE_MESSAGE_QUEUES
    if ((eType == OSEV_MSG) || (eType == OSEV_MSGQ)) {
    #endif
      *msgPP = (OStypeMsgP) 0;
    }
    #endif

    OSDecCallDepth();
    OSLeaveCritical();
    OSMsgRtn("OSWaitEvent",
             OSMakeStr("task %d timed out waiting for event %d.",
                       OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)),
             (OStypeErr) (OSERR_NOYIELD | OSERR_TIMEOUT));
  }
  OScTcbP->u2.runStatus.state = OSTCB_TASK_ELIGIBLE;
  #endif /* #if OSENABLE_TIMEOUTS */


  #if OSENABLE_FAST_SIGNALING
  if (OScTcbP->status.bits.state == OSTCB_TASK_WAITING) {
    OScTcbP->status.bits.state = OSTCB_TASK_ELIGIBLE;

    OScTcbP->status.bits.yielded = FALSE;

    #if OSENABLE_TIMEOUTS
    if ((OScTcbP->status.bits.state == OSTCB_TASK_WAITING) \
     && (OScTcbP->u1.ecbP != (OSgltypeEcbP) 0)) {
      OSDelDelay(OScTcbP);
    }
    #endif

    // If we got here because the event was signaled,
    //  we need to extract msgP from the task's tcb, and
    //  clear it in the tcb. Is clearing really reqd?
    #if OSENABLE_MESSAGES || OSENABLE_MESSAGE_QUEUES
    #if OSENABLE_MESSAGES && !OSENABLE_MESSAGE_QUEUES
    if (eType == OSEV_MSG) {
    #elif !OSENABLE_MESSAGES && OSENABLE_MESSAGE_QUEUES
    if (eType == OSEV_MSGQ) {
    #elif OSENABLE_MESSAGES && OSENABLE_MESSAGE_QUEUES
    if ((eType == OSEV_MSG) || (eType == OSEV_MSGQ)) {
    #endif
        *msgPP = OScTcbP->msgP;
        OScTcbP->msgP = (OStypeMsgP) 0;
    }
    #endif

    OSDecCallDepth();
    OSLeaveCritical();
    OSMsgRtn("OSWaitEvent",
             OSMakeStr("task %d was signaled waiting for event %d.",
                       OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)),
             (OStypeErr) (OSERR_NOYIELD | OSERR_SIGNALED));

  }
  #endif /* #if OSENABLE_FAST_SIGNALING */

  {
  // OK, so we didn't arrive due to a timeout or
  //  because the waited event got signaled. So now
  //  we must evaluate the event and either acquire
  //  it or make the task wait.
  // Always assume event is not available. Then let
  //  (certain) tests show that it is ... this is a
  //  winner most of the time.
  // This section redefined u.avail using the bitwise
  //  error codes available to us already
  u.avail = OSNOERR;

  // Figure out if the event is available or not.
  switch (eType) {

    #if OSENABLE_BINARY_SEMAPHORES
    // BINSEM -- simple way of seeing if binSem
    //  is available.
    // Also, these three tests (binSem, sem and
    //  msg) are identical and a good compiler will
    //  optimize them down to a single test when
    //  they're all 8-bit fields, as they are per
    //  default on PIC16.
    case OSEV_BINSEM:
      if (ecbP->event.binSem) {
          u.avail |= OSERR_AVAILABLE;
      }
      break;
    #endif

    #if OSENABLE_SEMAPHORES
    // SEM -- find out a priori if the task can
    //  proceed. Depending on the size of the
    //  semaphore, the test can be simple or a bit
    //  more complex.
    case OSEV_SEM:
      if (ecbP->event.sem) {
          u.avail |= OSERR_AVAILABLE;
      }
      break;
    #endif

    #if OSENABLE_MESSAGES
    // MSG -- if there's a defined msgP then the
    //  msg is available.
    case OSEV_MSG:
      if (ecbP->event.msgP) {
          u.avail |= OSERR_AVAILABLE;
      }
      break;
    #endif

    #if OSENABLE_EVENT_FLAGS
    // EFLAG -- a null pointer for efcbP is
    //  illegal, though it's OK for a task to wait
    //  on the event flag until it's created.
    case OSEV_EFLAG:
      #if OSENABLE_ERROR_CHECKING
      if (!ecbP->event.efcbP) {
        u.avail |= OSERR_SEVERE;
      }
      else
      #endif
      {
        /* here's where we determine whether    */
        /*   or not a task must wait the event  */
        /*   flags.                             */
        u2.eFlag = ecbP->event.efcbP->eFlag;
        mask     = va_arg(arg, OStypeEFlag);
        options  = va_arg(arg, OStypeOption);

        /* find out if the event we've been     */
        /*  waiting for has actually occurred.  */
        switch (options) {

          case OSANY_BITS:
            if (u2.eFlag & mask) {
              u.avail |= OSERR_AVAILABLE;
            }
            break;

          case OSEXACT_BITS:
            if (!(u2.eFlag ^ mask)) {
              u.avail |= OSERR_AVAILABLE;
            }
            break;

          case OSALL_BITS:
            if (!((u2.eFlag &= mask) ^ mask)) {
              u.avail |= OSERR_AVAILABLE;
            }
            break;

          default:
            break;
        } /* switch (options) */
      break;
      } /* case OSEV_EFLAG */
    #endif

    #if OSENABLE_MESSAGE_QUEUES
    // MSGQ -- a null pointer for mqcbP is 
    //  illegal, though it's OK for a task to wait
    //  on the msgq until it's created and filled
    //  with at least one msg.
    case OSEV_MSGQ:
      #if OSENABLE_ERROR_CHECKING
      if (!ecbP->event.mqcbP) {
        u.avail |= OSERR_SEVERE;
      }
      else
      #endif
      {
        if (ecbP->event.mqcbP->count) {
          u.avail |= OSERR_AVAILABLE;
        }
      }
      break;
    #endif

    default:
        break;
    }

    // u.avail is either 0 or OSERR_AVAIL or
    //  OSERR_SEVERE.

    #if OSENABLE_ERROR_CHECKING
    // if we have a severe error, e.g. we tried to wait
    //  an event that wasn't properly initialized, the
    //  safest thing is to simply stop the task.
    if (u.avail & OSERR_SEVERE) {
      OScTcbP->status.bits.state = OSTCB_TASK_STOPPED;
      OScTcbP->status.bits.yielded = FALSE;

      OSDecCallDepth();
      OSLeaveCritical();
      OSErrRtn("OSWaitEvent",
               OSMakeStr("task %d tried to wait non-existent event %d.",
                         OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)),
               (OStypeErr) OSERR_YIELD);
    }
    else
    #endif /* #if OSENABLE_ERROR_CHECKING */


    // OK, we got here by being ELIGIBLE and by being
    //  the current (running) state. If the event exists
    //  (as per the tests above), we can acquire it and
    //  continue without context-switching.
    // NOTE: since eFlags are fundamentally different
    //  from other events, as the bits "persist" and
    //  the user must explicitly clear event flag bits,
    //  there's no need to process them here.
    if (u.avail & OSERR_AVAILABLE) {

      if (OScTcbP->status.bits.yielded) {
          OScTcbP->status.bits.yielded = FALSE;

        switch (eType) {

          #if OSENABLE_BINARY_SEMAPHORES
          // BINSEM -- binSems are 0 once acquired.
          case OSEV_BINSEM:
            ecbP->event.binSem = 0;

            OSMsg("OSWaitEvent",
                  OSMakeStr("task % d acquired binary semaphore %d.",
                            OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)));
            break;
          #endif

          #if OSENABLE_MESSAGES
          // MSG -- if the msg is available without
          //  complications, then simply get it,
          //  clear it and we're done.
          case OSEV_MSG:
            *msgPP           = ecbP->event.msgP;
            ecbP->event.msgP =   (OStypeMsgP) 0;

            OSMsg("OSWaitEvent",
                  OSMakeStr("task % d acquired message %d (now %d).",
                            OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS), ecbP->event.msgP));
            break;
          #endif

          #if OSENABLE_MESSAGE_QUEUES
          // MSGQ -- using a local handle to speed
          //  things up, pass the message "up" and
          //  reflect its removal from the queue.
          case OSEV_MSGQ:
            u2.mqcbP         = ecbP->event.mqcbP;
            *msgPP           = *(u2.mqcbP->outPP);
            if ( ++(u2.mqcbP->outPP) == u2.mqcbP->endPP )
                u2.mqcbP->outPP = u2.mqcbP->beginPP;
            u2.mqcbP->count--;

            OSMsg("OSWaitEvent",
                  OSMakeStr("task % d acquired message %d from message queue.",
                            OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)));
            break;
          #endif

          #if OSENABLE_SEMAPHORES
          /* SEM -- sems are decremented when        */
          /*  acquired.                              */
          case OSEV_SEM:
            (ecbP->event.sem)--;

            OSMsg("OSWaitEvent",
                  OSMakeStr("task % d acquired semaphore %d (now %d).",
                            OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS), ecbP->event.sem));
            break;
          #endif

          default:
            break;
        } /* switch (eType) */

        OSMsg("OSWaitEvent",
              OSMakeStr("event %d is available.",
                        OSeID(ecbP, OSEVENTS)));
        u.err = OSERR_NOYIELD | OSERR_AVAILABLE;
      } /* if (OScTcbP->status.bits.yielded) */

      // If ( !OScTcbP->status.bits.yielded ), i.e. if
      //  we haven't yet yielded then we'll need to, and
      //  we'll come back later to "get" the event.
      else
      {
        OScTcbP->status.bits.yielded = TRUE;

        OSMsg("OSWaitEvent",
              OSMakeStr("event %d is available.",
                        OSeID(ecbP, OSEVENTS)));
        u.err = OSERR_YIELD;
      }
    }

    // If (!(u.avail & OSERR_AVAIL)), i.e. the event was
    //  not available, and so we have to wait it.
    else
    {

      #if !OSENABLE_TIMEOUTS
      OScTcbP->status.bits.state = OSTCB_TASK_WAITING;
      #else
      if (!timeout) {
        OScTcbP->status.bits.state = OSTCB_TASK_WAITING;
      }
      else if (OScTcbP->status.bits.state == OSTCB_TASK_ELIGIBLE) {
        OScTcbP->status.bits.state = OSTCB_TASK_WAITING;
        OScTcbP->dly.delay         = timeout;
        OScTcbP->u1.ecbP            = ecbP;
        OSInsDelay(OScTcbP);
      }
      #endif

      OScTcbP->status.bits.yielded = TRUE;

      OSInsPrio(OScTcbP, &(ecbP->tcbP));

      OSMsg("OSWaitEvent",
            OSMakeStr("task %d now waiting event %d.",
                      OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)));
      u.err = OSERR_YIELD;
    }

    va_end(arg);
  } /* not OSTCB_TASK_WAITING nor OSTCB_TASK_WAITING */

  OSDecCallDepth();
  OSLeaveCritical();

  return (u.err);
}

#include <salvoscg.h>
#undef __OSWAITEVENT_EVENT_C



#else /* #if OSCOMBINE_EVENT_SERVICES */
/************************************************************
****                                                     ****
**                                                         **
Event services for OSCOMBINE_EVENT_SERVICES is FALSE

In this case, OS_WaitXyz() calls OSWaitEvent().


**                                                         **
****                                                     ****
************************************************************/


/************************************************************
****                                                     ****
**                                                         **
OSWaitEvent(ecbP, eventType, timeout)

Make current task wait event.

A task can enter OSWaitEvent() in several different ways,
and hence in various different states. It could be:

  * ELIGIBLE ("fell in" as part of normal wait-on-event
     task execution)
  * WAITING (resumed via scheduler, task was waiting
     event when it was signaled, may have an unexpired
     timeout associated with it)
  * SIGNALED (same as WAITING, but for fast signaling)
  * TIMED_OUT (resumed via scheduler because task
     timed out while waiting event)

These different conditions require different actions, both
within OSWaitEvent() and in the OS_WaitXyz() wrapper. Hence
return codes are used for various purposes.

The yielded bit is used to guarantee one and only one
context switch whenever OS_WaitXyz() is called. This
improves system responsiveness (because a high-priority task
may have become eligible between context switches in the
current task), and it also avoids the problem that occurs
if you don't context switch if the event has been signaled
-- in that case, bombarding the event with signals can cause
the task to never context switch, which is a definite no-no.

NOTE: enter with interrupts disabled!

**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSWAITEVENT_EVENT_C
#include <salvomcg.h>
#endif

#if !OSENABLE_TIMEOUTS
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitEvent(OStypeEcbP    ecbP,
                      OStypeBoolean avail)
OSMONITOR_KEYWORD_POST
#else
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitEvent(OStypeEcbP    ecbP,
                      OStypeBoolean avail,
                      OStypeDelay   timeout)
OSMONITOR_KEYWORD_POST
#endif
{
  OStypeErr err;


  // Task enters as ELIGIBLE, WAITING, SIGNALED or
  //  TIMED_OUT.
  
  OSIncCallDepth();

  #if OSENABLE_TIMEOUTS
  // Did we arrive due to a timeout?
  if (OScTcbP->status.bits.state == OSTCB_TASK_TIMED_OUT) {
    
    // Now that runStatus holds timeout flag, we can
    //  change the task's state to ELIGIBLE.
    OScTcbP->status.bits.state = OSTCB_TASK_ELIGIBLE;

    // We yielded when we waited with a timeout, so
    //  it's time to clear the flag.
    OScTcbP->status.bits.yielded = FALSE;

    // Return through to wrapper without a context
    //  switch, and with the timeout flagged.
    // No need to remove tcb from delayQ, as timeout
    //  has already occurred.
    // TIMED_OUT tasks exit here ...
    OSDecCallDepth();
    OSMsgRtn("OSWaitEvent",
             OSMakeStr("task %d timed out waiting for event %d.",
                       OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)),
             (OStypeErr) (OSERR_NOYIELD | OSERR_TIMEOUT));
  }

  // Since timeouts are enabled and we got here without
  //  being TIMED_OUT, we need to reset / override the
  //  runStatus timedout indicator. O/wise subsequent
  //  calls to OSWaitEvent() that succeed with the event
  //  available will still see a persistent timed-out
  //  indicator, which is wrong.
  // N.B runStatus is only valid while a task is running.
  else /* if (OScTcbP->status.bits.state == OSTCB_TASK_TIMED_OUT) */
  {
    OScTcbP->u2.runStatus.state = OSTCB_TASK_ELIGIBLE;
  }
  #endif


	// Task must be ELIGIBLE, WAITING or SIGNALED.
  // The event is available. If we were previously
  //  waiting it, either because it was previously
  //  unavailable OR we context-switched unconditionally,
  //  then we can proceed without any further context
  //  switches. O/wise we need to context-switch (once).
	// SIGNALED events always enter here. ELIGIBLE and
	//  WAITING events will enter here if the event  is
	//  available.
  if (avail) {
    // If we have already context-switched as part of
    //  waiting the event, then proceed with processing
    //  the event.
    if (OScTcbP->status.bits.yielded) {
      OScTcbP->status.bits.yielded = FALSE;

      // Since this is where we end up after waiting
      //  an event with a timeout (and we didn't time
      //  out), this is where we have to remove the
      //  task from the delayQ and clear its link to
      //  the task it was waiting.
      // Note that removal from the delayQ will
      //  always succeed if the task was waiting the
      //  event with a timeout, i.e. it's WAITING or
      //  SIGNALED and the timeout hasn't expired.
      #if OSENABLE_TIMEOUTS
      if (OScTcbP->u1.ecbP != (OSgltypeEcbP) 0) {
        OScTcbP->u1.ecbP = (OSgltypeEcbP) 0;
        OSDelDelay(OScTcbP);
      }
      #endif

      // When fast signaling, we need to tell the
      //  caller (i.e. OSWaitXyz()) that this was a
      //  fast-signaled event. O/wise it's just a
      //  normally-signaled event.
      #if !OSENABLE_FAST_SIGNALING
      err = OSERR_AVAILABLE;
      #else
	    if (OScTcbP->status.bits.state == OSTCB_TASK_SIGNALED) {
	      err = OSERR_SIGNALED;
	    }
	    else {
	      err = OSERR_AVAILABLE;
	    }
      #endif

      // Since we're done waiting and ready to
      //  proceed, the task becomes ELIGIBLE.
      OScTcbP->status.bits.state = OSTCB_TASK_ELIGIBLE;

      // Return through to wrapper without a context
      //  switch, and with the the fact that the
      //  event has been signaled and is available to
      // us. SIGNALED tasks always exit here.
      OSDecCallDepth();
      OSMsgRtn("OSWaitEvent",
       OSMakeStr("event %d is available.",
                 OSeID(ecbP, OSEVENTS)),
                 (OStypeErr) (OSERR_NOYIELD | err));
    }

    // O/wise do a context switch first. We'll proceed
    //  if we're (still) the highest-priority eligible
    //  task. O/wise a higher-priority task will get
    //  to run.
    else {
      // We'll yield just this once ...
      OScTcbP->status.bits.yielded = TRUE;

      // Return through to wrapper with a simple
      //  context switch.
      OSDecCallDepth();
      OSMsgRtn("OSWaitEvent",
               OSMakeStr("event %d is available.",
                         OSeID(ecbP, OSEVENTS)),
               (OStypeErr) OSERR_YIELD);
    }
  }


  // Task must be ELIGIBLE or WAITING.
  // Since the event is not available, the task needs to
  //  be enqueued into the eventQ with an optional timeout
  //   (i.e. timeout != 0).
  // Note that for multiple tasks WAITING the same event,
  //  it's quite possible for a low-priority task to
  //  return here after said event is signaled, due to a
  //  higher-priority task "snatching" the signaled event.
  else
  {
    #if !OSENABLE_TIMEOUTS
    // Setup the task to wait the event.
    OScTcbP->status.bits.state = OSTCB_TASK_WAITING;

    #else
    // If a timeout was specified we must store the
    //  ecbP for later use by OSTimer() and put the
    //  task into the delay queue. When dispatched,
    //  a task with a non-zero ecbP is waiting the
    //  event with a timeout.
    if (!timeout) {
        OScTcbP->status.bits.state = OSTCB_TASK_WAITING;
    }
    // To reach this, we must be either ELIGIBLE or
    //  WAITING. If ELIGIBLE, then we begin the
    //  wait-on-event-with-timeout process. If
    //  WAITING, we're still waiting, and so we
    //  don't change a thing ...
    else if (OScTcbP->status.bits.state == OSTCB_TASK_ELIGIBLE) {
        OScTcbP->status.bits.state = OSTCB_TASK_WAITING;
        OScTcbP->dly.delay         = timeout;
        OScTcbP->u1.ecbP           = ecbP;
        OSInsDelay(OScTcbP);
    }
    #endif

    // We'll now wait until the task is signaled, with
    //  an optional timeout. Since the event was not
    //  available when we waited it, we'll force a
    //  context switch (via OSERR_YIELD below) and mark
    //  the fact that a context switch has occurred, so
    //  that when we return to this function, we can
    //  wrap up and continue task execution without
    //  further context switches.
    OScTcbP->status.bits.yielded = TRUE;

    // Make the task wait the event. OK to repeat for
    //  a task that entered as WAITING because said
    //  task left the eventQ when it was signaled! IOW,
    //  a task that entered as WAITING must re-wait
    //  the event.
    OSInsPrio(OScTcbP, &(ecbP->tcbP));

    // Return through wrapper with a context switch.
    //  Task is now WAITING with optional timeout.
    OSDecCallDepth();
    OSMsgRtn("OSWaitEvent",
             OSMakeStr("task %d now waiting event %d.",
                       OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)),
             (OStypeErr) OSERR_YIELD);
  }
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSWAITEVENT_EVENT_C
#endif

#endif /* #if OSCOMBINE_EVENT_SERVICES */

#endif /* OSENABLE_EVENTS */
