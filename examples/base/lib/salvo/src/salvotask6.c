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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvotask6.c,v $
$Author: aek $
$Revision: 3.14 $
$Date: 2008-04-27 14:45:27-07 $

Function to set the priority of the specified task.

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS


/************************************************************
****                                                     ****
**                                                         **
OSSetPrioTask(tcbP, prio)

Change the specified task's priority.

Note: Currently only works from mainline code and tasks, not
from within interrupts.

Note: Currently supports changing a waiting task's priority
only if timeouts are enabled.

Note: stopped and delayed tasks can simply have their
priorities changed without worrying about queues.

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_ARRAYS && !OSDISABLE_TASK_PRIORITIES

#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSETPRIOTASK_TASK6_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSSetPrioTask( OStypeTcbP tcbP,
                         OStypePrio prio )
OSMONITOR_KEYWORD_POST
{
  OStypeErr err;


	// Assume an error, set to OSNOERR based on actions
	//  below.
	err = OSERR;

  OSEnterCritical();
  OSIncCallDepth();
                              
  // If the task's state is valid and it can successfully
  //  be removed from whatever queue it's in, then go
  //  ahead and remove it. O/wise we can't continue.
  // Delayed tasks don't have to come out of the OSdelayQ
  //  'cause they're enqueued based on delay, not
  //  priority.
  if (OSTaskUsed(tcbP)) {
    if (OSDelTaskQ(tcbP, FALSE) == OSNOERR) {

      // Task is now an orphan (not in any queue) -- it's
      //  safe to change its priority.
      #if OSENABLE_ERROR_CHECKING
      OSInitPrioTask(tcbP, prio);
      #else
      tcbP->status.bits.prio = prio;
      #endif

      // Now insert the task into the appropriate queue.
      err = OSInsTaskQ(tcbP);
    }
  }

  OSDecCallDepth();
  OSLeaveCritical();

  return err;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSSETPRIOTASK_TASK6_C
#endif

#endif

#endif /* #if OSENABLE_TASKS */

