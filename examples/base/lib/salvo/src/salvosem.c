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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvosem.c,v $
$Author: aek $
$Revision: 3.20 $
$Date: 2008-04-27 14:45:31-07 $

Functions to create, wait and signal a counting semaphore.

************************************************************/

#include <salvo.h>

#if OSENABLE_SEMAPHORES

/************************************************************
****                                                     ****
**                                                         **
Sem services when OSCOMBINE_EVENT_SERVICES is FALSE.

**                                                         **
****                                                     ****
************************************************************/
#if !OSCOMBINE_EVENT_SERVICES

/************************************************************
** OSCreateSem(ecbP, sem)                                  **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSCREATESEM_SEM_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSCreateSem( OStypeEcbP ecbP,
                       OStypeSem  sem )
OSMONITOR_KEYWORD_POST
OSCREATESEM_ATTR
{
  OSEnterCritical();
  OSIncCallDepth();

  ecbP->tcbP      = (OSgltypeTcbP) 0;
  ecbP->event.sem = sem;
  #if OSUSE_EVENT_TYPES
  ecbP->type      = OSEV_SEM;
  #endif

  OSDecCallDepth();
  OSLeaveCritical();

  return OSNOERR;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSCREATESEM_SEM_C
#endif


/************************************************************
** OSWaitSem(ecbP, timeout)                                **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSWAITSEM_SEM_C
#include <salvomcg.h>
#endif

#if !OSENABLE_TIMEOUTS
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitSem( OStypeEcbP  ecbP )
OSMONITOR_KEYWORD_POST
#else
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitSem( OStypeEcbP  ecbP,
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
    #if !OSBIG_SEMAPHORES
    u.avail = ecbP->event.sem;
    #else
    u.avail = FALSE;
    if (ecbP->event.sem > 0) {
      u.avail++;
    }
    #endif
  }


  #if !OSENABLE_TIMEOUTS
  u.err = OSWaitEvent(ecbP, u.avail);
  #else
  u.err = OSWaitEvent(ecbP, u.avail, timeout);
  #endif


  if ((u.err & OSERR_AVAILABLE) != 0) {
  (ecbP->event.sem)--;

    OSMsg("OSWaitSem",
          OSMakeStr("task % d acquired semaphore %d (now %d).",
          OStID(OScTcbP, OSTASKS), OSeID(ecbP, OSEVENTS), ecbP->event.sem));
  }


  OSDecCallDepth();
  OSLeaveCritical();

  return (u.err);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSWAITSEM_SEM_C
#endif


/************************************************************
** OSSignalSem(ecbP)                                       **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSIGNALSEM_SEM_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSSignalSem( OStypeEcbP ecbP )
OSMONITOR_KEYWORD_POST
OSSIGNALSEM_ATTR
{
  union {
      OStypeErr  err;
      OStypeTcbP tcbP;
  } u;


  OSEnterCritical();
  OSIncCallDepth();

  #if OSENABLE_BOUNDS_CHECKING
  if ((ecbP < OSECBP(1)) || (ecbP > OSECBP(OSEVENTS_LIMIT))) {
    OSWarn("OSSignalSem",
           OSMakeStr("semaphore %d nonexistent or invalid.",
                     OSeID(ecbP, OSEVENTS)));

    u.err = OSERR_BAD_P;
  }

  else
  #endif
  {
    #if OSENABLE_ERROR_CHECKING && OSUSE_EVENT_TYPES
    if (ecbP->type != OSEV_SEM) {
      OSWarn("OSSignalSem",
             OSMakeStr("event %d is not a sempahore.",
                       OSeID(ecbP, OSEVENTS)));

      u.err = OSERR_EVENT_BAD_TYPE;
    }

    else
    #endif
    {
      #if !OSENABLE_FAST_SIGNALING
      if (ecbP->event.sem >= MAX_SEM) {
        OSWarn("OSSignalSem",
               OSMakeStr("semaphore %d maxed out.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_FULL;
      }

      else {
        (ecbP->event.sem)++;

        u.tcbP = ecbP->tcbP;

        if (u.tcbP) {
          OSInsSigQ(u.tcbP, ecbP);
        }

        u.err = OSNOERR;
      }
      #else
      if (!ecbP->tcbP && (ecbP->event.sem >= MAX_SEM)) {
        OSWarn("OSSignalSem",
               OSMakeStr("semaphore %d maxed out.",
                         OSeID(ecbP, OSEVENTS)));

        u.err = OSERR_EVENT_FULL;
      }

      else
      {
        u.tcbP = ecbP->tcbP;

        if (u.tcbP) {
          u.tcbP->status.bits.state = OSTCB_TASK_SIGNALED;
          OSInsSigQ(u.tcbP, ecbP);
        }
        else {
          (ecbP->event.sem)++;
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
#undef __OSSIGNALSEM_SEM_C
#endif

#endif /* #if !OSCOMBINE_EVENT_SERVICES */

#endif /* #if OSENABLE_SEMAPHORES */
