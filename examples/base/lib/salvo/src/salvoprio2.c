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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoprio2.c,v $
$Author: aek $
$Revision: 3.8 $
$Date: 2008-04-27 14:45:33-07 $

Function to get a task's priority.

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS


#if !OSUSE_ARRAYS
/************************************************************
****                                                     ****
**                                                         **
OSGetPrioTask()

Return the priority of the specified task.

Used by OSGetPrio().

**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSGETPRIOTASK_PRIO_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypePrio OSGetPrioTask( OStypeTcbP tcbP )
OSMONITOR_KEYWORD_POST
OSGETPRIOTASK_ATTR
{
  OStypePrio prio;


  OSEnterCritical();
  OSIncCallDepth();

  prio = tcbP->status.bits.prio;

  OSDecCallDepth();
  OSLeaveCritical();

  return prio;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSGETPRIOTASK_PRIO_C
#endif

#endif /* #if !OSUSE_ARRAYS */

#endif /* #if OSENABLE_TASKS */

