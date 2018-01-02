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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvotask5.c,v $
$Author: aek $
$Revision: 3.14 $
$Date: 2008-04-27 14:45:28-07 $

Function to get the specified task's state.

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS

/************************************************************
****                                                     ****
**                                                         **
OSGetStateTask()

Return the state of the current task. Used by OSGetState().

NOTE: The usefulness of OSGetState() is unclear.

**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSGETSTATETASK_TASK5_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeState OSGetStateTask( OStypeTcbP tcbP )
OSMONITOR_KEYWORD_POST
OSGETSTATETASK_ATTR
{
  OStypeState state;

  OSEnterCritical();
  OSIncCallDepth();

  if (OSTaskRunning(tcbP)) {
    state = OSTCB_TASK_RUNNING;
  }
  else {
    state = tcbP->status.bits.state;
  }

  OSDecCallDepth();
  OSLeaveCritical();

  return state;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSGETSTATETASK_TASK5_C
#endif

#endif /* #if OSENABLE_TASKS */

