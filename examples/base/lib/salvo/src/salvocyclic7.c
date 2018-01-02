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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvocyclic7.c,v $
$Author: aek $
$Revision: 3.7 $
$Date: 2008-04-27 14:45:42-07 $

Functions to handle cyclic timers.

************************************************************/

#include <salvo.h>

#if OSENABLE_CYCLIC_TIMERS

/************************************************************ 
****                                                     ****
**                                                         **
OSCycTmrRunning()

Returns TRUE if the cyclic timer is running, o/wise returns
FALSE.

Does not return a meaningful value if called from within the
cycTmr itself ...

**                                                         **
****                                                     ****
************************************************************/
#define __OSCYCTMRRUNNING_CYCLIC7_C
#include <salvomcg.h>

OSMONITOR_KEYWORD_PRE
OStypeErr OSCycTmrRunning ( OStypeTcbP    tcbP )
OSMONITOR_KEYWORD_POST                           
{
  OStypeErr running;


  OSEnterCritical();
  OSIncCallDepth();  
  
  if ((tcbP->status.bits.state == OSTCB_CYCLIC_TIMER) \
    && (tcbP->status.bits.yielded == TRUE)) {
    running = TRUE;
  }
  else {
    running = FALSE;
  }
                                             
  OSDecCallDepth();    
  OSLeaveCritical();
  
  return running;
}

#include <salvoscg.h>
#undef __OSCYCTMRRUNNING_CYCLIC7_C

#endif /* #if OSENABLE_CYCLIC_TIMERS */



