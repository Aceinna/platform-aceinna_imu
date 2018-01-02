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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvostop.c,v $
$Author: aek $
$Revision: 3.12 $
$Date: 2008-04-27 14:45:30-07 $

Function to stop a task.

************************************************************/

#include <salvo.h>

#if !OSENABLE_DELAYS


/************************************************************ 
****                                                     ****
**                                                         **
OSStop()

Stop the current task.

Note: Context-switch immediately after OSStop()!

Note: OSStop() is used only when delays are not enabled. If
delays are enabled, OS_Stop() calls OSDelay(0).

**                                                         **
****                                                     ****
************************************************************/
#define __OSSTOP_STOP_C
#include <salvomcg.h>

OSMONITOR_KEYWORD_PRE
void OSStop( void )
OSMONITOR_KEYWORD_POST
{
  OSEnterCritical();
  OSIncCallDepth();    
  
  // Make the task STOPPED.
  OScTcbP->status.bits.state = OSTCB_TASK_STOPPED;

  // And tell about it.
  OSDecCallDepth();
  OSLeaveCritical();
  OSMsg("OSStop", 
        OSMakeStr("task %d stopped.",
                  OStID(OScTcbP, OSTASKS)));
}

#include <salvoscg.h>
#undef __OSSTOP_STOP_C


#endif /* #if !OSENABLE_DELAYS */
