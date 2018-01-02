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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvotask3.c,v $
$Author: aek $
$Revision: 3.15 $
$Date: 2008-04-27 14:45:29-07 $

Function to destroy the specified task.

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS


/************************************************************
****                                                     ****
**                                                         **
OSDestroyTask(tcbP, OSEVENTS)

Destroys the specified task regardless of its state.

Note: Does not re-initialize or completely clear all of the
tcb fields. It clears only the status field. Therefore when
parsing the tcb, if the task is destroyed than all of its
other fields must be ignored.

Note: Currently only works from mainline code and tasks, not
from within interrupts.

**                                                         **
****                                                     ****
************************************************************/

#if !OSUSE_ARRAYS

#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSDESTROYTASK_TASK3_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
#if OSENABLE_EVENTS
OStypeErr OSDestroyTask( OStypeTcbP tcbP,
                         OStypeID   events )
#else
OStypeErr OSDestroyTask( OStypeTcbP tcbP )
#endif
OSMONITOR_KEYWORD_POST
{
  #if OSENABLE_EVENTS
  OStypeID i;
  #endif


  OSEnterCritical();

  #if OSENABLE_DELAYS || OSENABLE_TIMEOUTS
  // Task may be delayed.
  OSDelDelay(tcbP);
  #endif

  #if OSENABLE_SIGQ
  // Task may be in transition via the sigQ.
  OSDelPrioQ(tcbP, &OSsigQoutP);
  #endif

  // Task may be eligible.
  OSDelPrioQ(tcbP, &OSeligQP);

  #if OSENABLE_EVENTS
  // Task may be waiting for an event.
  for (i=1; i<=events; i++) {
    OSDelPrioQ(tcbP, &(OSECBP(i))->tcbP);
  }
  #endif

  // Finally, zero out the task's entire status field.
  tcbP->status.value = OSTCB_DESTROYED;

  OSLeaveCritical();

  // And exit.
  OSMsgRtn("OSDestroyTask",
           OSMakeStr("task %d destroyed.", OStID(tcbP, OSTASKS)),
           (OStypeErr) OSNOERR);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSDESTROYTASK_TASK3_C
#endif

#endif

#endif /* #if OSENABLE_TASKS */

