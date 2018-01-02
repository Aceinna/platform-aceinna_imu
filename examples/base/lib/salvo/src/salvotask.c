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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvotask.c,v $
$Author: aek $
$Revision: 3.18 $
$Date: 2008-04-27 14:45:30-07 $

Function to manipulate a specified task.

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS


/************************************************************
****                                                     ****
**                                                         **
OSStartTask(tcbP)

Start a task. Task must have been created beforehand via
OSCreateTask(), and should be OSTCB_TASK_STOPPED. The task will
execute from the scheduler once it becomes the highest-
priority eligible task.

Returns:     OSNOERR if successful
            OSERR if specified task doesn't exist

**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSTARTTASK_TASK_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSStartTask( OStypeTcbP tcbP )
OSMONITOR_KEYWORD_POST
OSSTARTTASK_ATTR
{
  // Start stack checking
  OSEnterCritical();
  OSIncCallDepth();

  // Punt if tcbP is clearly bad.
  #if OSENABLE_BOUNDS_CHECKING
  if ((tcbP < OSTCBP(1)) || (tcbP > OSTCBP(OSTASKS_LIMIT))) {
    OSDecCallDepth();
    OSLeaveCritical();    
    OSWarnRtn("OSStartTask",
              OSMakeStr("task %d nonexistent or invalid.",
                        OStID(tcbP, OSTASKS)), 
              (OStypeErr) OSERR_BAD_P);
  }
  #endif

  // tcbP must be valid and task must be
  //  initialized properly or it can't be made
  //  ready.
  if (tcbP->status.bits.state != OSTCB_TASK_STOPPED) {
    OSDecCallDepth();
    OSLeaveCritical();
    OSWarnRtn("OSStartTask",
              OSMakeStr("unable to start task %d.", 
                        OStID(tcbP, OSTASKS)),
              (OStypeErr) OSERR);
  }

  // Make task OSTCB_TASK_ELIGIBLE and enqueue it into
  //  signaled queue.           
  #if OSCALL_OSSTARTTASK == OSFROM_BACKGROUND
  OSInsElig(tcbP);
  #else
  tcbP->status.bits.state = OSTCB_TASK_ELIGIBLE;
  if (!OSsigQinP) {
    OSsigQoutP             = tcbP;
  }
  else {
    OSsigQinP->u2.nextTcbP = tcbP;
  }
  OSsigQinP                = tcbP;
  tcbP->u2.nextTcbP        = (OSgltypeTcbP) 0;
  #endif

  // Finish stack checking.
  OSDecCallDepth();
  OSLeaveCritical();

  // Success, task was started.
  return OSNOERR;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSSTARTTASK_TASK_C
#endif

#endif /* #if OSENABLE_TASKS */

