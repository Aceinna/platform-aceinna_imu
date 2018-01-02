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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvobinsem.c,v $
$Author: aek $
$Revision: 3.21 $
$Date: 2008-04-27 14:45:46-07 $

Functions to create, wait and signal a binary semaphore.

************************************************************/

#include <salvo.h>


#if OSENABLE_BINARY_SEMAPHORES

/************************************************************
****                                                     ****
**                                                         **
BinSem services when OSCOMBINE_EVENT_SERVICES is FALSE.

See also event.c for documented combined version.

The versions herein should function identically, albeit
without reduced call graphs.

**                                                         **
****                                                     ****
************************************************************/
#if !OSCOMBINE_EVENT_SERVICES

/************************************************************
** OSCreateBinSem(ecbP, binSem)                            **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSCREATEBINSEM_BINSEM_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSCreateBinSem( OStypeEcbP   ecbP,
                          OStypeBinSem binSem )
OSMONITOR_KEYWORD_POST
OSCREATEBINSEM_ATTR
{
  // First-level function, so critical section must
  //  be protected ...                             
  OSEnterCritical();
  OSIncCallDepth();

  // Very simple -- just initialize all fields.
  ecbP->tcbP         = (OSgltypeTcbP) 0;
  ecbP->event.binSem = (OStypeBinSem) (binSem & 0x01);
  #if OSUSE_EVENT_TYPES
  ecbP->type         = OSEV_BINSEM;
  #endif

  // Clean up and get out.
  OSDecCallDepth();
  OSLeaveCritical();

  return OSNOERR;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSCREATEBINSEM_BINSEM_C
#endif

/************************************************************
** OSWaitBinSem(ecbP, timeout)                             **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSWAITBINSEM_BINSEM_C
#include <salvomcg.h>
#endif

#if !OSENABLE_TIMEOUTS
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitBinSem( OStypeEcbP  ecbP )
OSMONITOR_KEYWORD_POST
#else
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitBinSem( OStypeEcbP  ecbP,
                        OStypeDelay timeout )
OSMONITOR_KEYWORD_POST
#endif
{
  union {
    OStypeErr     err;
    OStypeBoolean avail;
  } u;


  // First-level function, so critical section must
  //  be protected ...
  OSEnterCritical();
  OSIncCallDepth();


  // Pass info on whether or not the event is       
  //  available to OSWaitEvent(). When fast signaling,
  //  we force OSWaitEvent() to treat the event as if  
  //  it's unconditionally available, because a task  
  //  that's been SIGNALED already "got" the event.  
  #if OSENABLE_FAST_SIGNALING
  if (OScTcbP->status.bits.state == OSTCB_TASK_SIGNALED) {
    u.avail = TRUE;
  }
  else
  #endif
  {
    u.avail = ecbP->event.binSem;
  }


  // OSWaitEvent() is the real engine here -- it
  //  detects how we got here (by being ELIGIBLE,
  //  WAITING, SIGNALED or TIMED_OUT) and what
  //  course of action we should take. All it needs
  //  from us is to know whether or not the event in
  //  question is available.
  #if !OSENABLE_TIMEOUTS
  u.err = OSWaitEvent(ecbP, u.avail);
  #else
  u.err = OSWaitEvent(ecbP, u.avail, timeout);
  #endif


  // Now that we're back, we'll use the bit-wise
  //  error flags that OSWaitEvent() returns in order
  //  to do necessary post-processing.

  // In the case where an event had been previously
  //  signaled and now the task has waited it, we
  //  must "get" the event properly.  This is an
  //  example of how an event is "automatically"
  //  cleared via a call to OS_WaitXyz().
  // Note that SIGNALED tasks don't need to do this.
  if ((u.err & OSERR_AVAILABLE) != 0) {
    ecbP->event.binSem = 0;

    OSMsg("OSWaitBinSem",
          OSMakeStr("task % d acquired binary semaphore %d.",
                    OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS)));
  }

  // Clean up.
  OSDecCallDepth();
  OSLeaveCritical();


  // Return to wrapper with OSWaitEvent()'s return code.
  return (u.err);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSWAITBINSEM_BINSEM_C
#endif

/************************************************************
** OSSignalBinSem(ecbP)                                    **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSIGNALBINSEM_BINSEM_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSSignalBinSem( OStypeEcbP ecbP )
OSMONITOR_KEYWORD_POST
OSSIGNALBINSEM_ATTR
{
  union {
    OStypeErr  err;
    OStypeTcbP tcbP;
  } u;


  // First-level function, so critical section must
  //  be protected ...
  OSEnterCritical();
  OSIncCallDepth();


  #if OSENABLE_BOUNDS_CHECKING
  // Punt if ecbP appears bad ...
  if ((ecbP < OSECBP(1)) || (ecbP > OSECBP(OSEVENTS_LIMIT))) {
    OSWarn("OSSignalBinSem",
           OSMakeStr("binary semaphore %d nonexistent or invalid.",
                     OSeID(ecbP, OSEVENTS)));

    u.err = OSERR_BAD_P;
  }
  else
  #endif
  {
    #if OSENABLE_ERROR_CHECKING && OSUSE_EVENT_TYPES
    // Punt if event being operated on is not a
    //  binsem.
    if (ecbP->type != OSEV_BINSEM) {
      OSWarn("OSSignalBinSem",
             OSMakeStr("event %d is not a binary semaphore.",
                       OSeID(ecbP, OSEVENTS)));

      u.err = OSERR_EVENT_BAD_TYPE;
    }

    else
    #endif
    {
      #if !OSENABLE_FAST_SIGNALING
      // With slow signaling, the binsem won't be
      //  cleared until the receiving task runs.
      //  Therefore we don't care if a task is
      //  waiting the event or not.
      if (ecbP->event.binSem) {
        OSWarn("OSSignalBinSem",
               OSMakeStr("binary semaphore %d already set.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_FULL;
      }
      // Just set the binsem and if a task was
      //  waiting, put it into the signaled queue.
      else {
        ecbP->event.binSem = 1;

        u.tcbP = ecbP->tcbP;

        if (u.tcbP) {
          OSInsSigQ(u.tcbP, ecbP);
        }

        u.err = OSNOERR;
      }

      #else
      // OSENABLE_FAST_SIGNALING case.
      // This is a little different ... a binsem
      //  is full only if it's set and no task is
      //  waiting.
      if (!ecbP->tcbP && ecbP->event.binSem) {
        OSWarn("OSSignalBinSem",
               OSMakeStr("binary semaphore %d already set.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_FULL;
      }

      // If a task is waiting, then put it into
      //  the signaled queue as SIGNALED, else
      //  just set the binsem.
      else {
        u.tcbP = ecbP->tcbP;

        if (u.tcbP) {
          u.tcbP->status.bits.state = OSTCB_TASK_SIGNALED;
          OSInsSigQ(u.tcbP, ecbP);
        }
        else {
          ecbP->event.binSem = 1;
        }

        u.err = OSNOERR;
      }
      #endif /* #if !OSENABLE_FAST_SIGNALING */
    }
  }

  // Clean up and get out.
  OSDecCallDepth();
  OSLeaveCritical();

  return (u.err);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSSIGNALBINSEM_BINSEM_C
#endif

#endif /* #if !OSCOMBINE_EVENT_SERVICES */

#endif /* #if OSENABLE_BINARY_SEMAPHORES */

