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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvocyclic6.c,v $
$Author: aek $
$Revision: 3.11 $
$Date: 2008-04-27 14:45:43-07 $

Functions to handle cyclic timers.

************************************************************/

#include <salvo.h>

#if OSENABLE_CYCLIC_TIMERS

/************************************************************ 
****                                                     ****
**                                                         **
OSResetCycTmr()

Reset a cyclic timer. Remove it from the delay queue (if 
required), re-initialize its delay field, and re-enqueue it.

**                                                         **
****                                                     ****
************************************************************/
#define __OSRESETCYCTMR_CYCLIC6_C
#include <salvomcg.h>

OSMONITOR_KEYWORD_PRE
OStypeErr OSResetCycTmr ( OStypeTcbP    tcbP )
OSMONITOR_KEYWORD_POST                           
{
  #if OSENABLE_ERROR_CHECKING
  if (tcbP->status.bits.state != OSTCB_CYCLIC_TIMER) {
    OSWarnRtn("OSResetCycTmr",
              OSMakeStr("not a cyclic timer."),
              (OStypeErr) OSERR_BAD_CT);   
  }
  #endif
  
  OSEnterCritical();
  OSIncCallDepth();  
  
  // If it's running, remove it from the delay queue.
  if (tcbP->status.bits.yielded == TRUE) {
    OSDelDelay(tcbP);
  }

  // (Re-)set the delay field to be the period.
  tcbP->dly.delay = tcbP->u1.period;

  // Re-enqueue the cycTmr into the delay queue.
  tcbP->status.bits.yielded = TRUE;
  OSInsDelay(tcbP);
                                             
  OSDecCallDepth();    
  OSLeaveCritical();
  OSMsgRtn("OSResetCycTmr", 
           OSMakeStr("cyclic timer %d reset.", OStID(tcbP, OSTASKS)), 
           (OStypeErr) OSNOERR);
}

#include <salvoscg.h>
#undef __OSRESETCYCTMR_CYCLIC6_C

#endif /* #if OSENABLE_CYCLIC_TIMERS */



