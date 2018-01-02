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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvocyclic2.c,v $
$Author: aek $
$Revision: 3.12 $
$Date: 2008-04-27 14:45:44-07 $

Functions to handle cyclic timers.

************************************************************/

#include <salvo.h>

#if OSENABLE_CYCLIC_TIMERS

/************************************************************ 
****                                                     ****
**                                                         **
OSStartCycTmr()

Start a cyclic timer that was already created, or that is
stopped.

**                                                         **
****                                                     ****
************************************************************/
#define __OSSTARTCYCTMR_CYCLIC2_C
#include <salvomcg.h>

OSMONITOR_KEYWORD_PRE
OStypeErr OSStartCycTmr ( OStypeTcbP    tcbP )
OSMONITOR_KEYWORD_POST                           
{
  OStypeErr err;


  // Punt if tcbP is clearly bad.
  #if OSENABLE_BOUNDS_CHECKING
  if ((tcbP < OSTCBP(1)) || (tcbP > OSTCBP(OSTASKS_LIMIT))) {
    OSWarnRtn("OSStartCycTmr",
              OSMakeStr("task %d nonexistent or invalid.",
                        OStID(tcbP, OSTASKS)),
              (OStypeErr) OSERR_BAD_P);
  }
  
  // v7.87 PL2 and earlier PICC compilers have a
  //  PIC12 bug that shows up if the bounds checking
  //  above is disabled. This is a simple fix.
  //  Problem happens when generating libraries.
  #elif OSWORKAROUND_HT_PICC_1
  if (tcbP == (OSgltypeTcbP) 0) {
    OSWarnRtn("OSStartCycTmr",
              OSMakeStr("task %d nonexistent or invalid.",
                        OStID(tcbP, OSTASKS)),
              (OStypeErr) OSERR_BAD_P);
  }
  #endif
  
  #if OSENABLE_ERROR_CHECKING
  if (tcbP->status.bits.state != OSTCB_CYCLIC_TIMER) {
    OSWarnRtn("OSStartCycTmr",
              OSMakeStr("not a cyclic timer."),
              (OStypeErr) OSERR_BAD_CT);
  }
  #endif
  
  
  OSEnterCritical();
  OSIncCallDepth();  
  
  // Enqueue the cycTmr into the delay queue.
  if (tcbP->status.bits.yielded == FALSE) {
    tcbP->status.bits.yielded = TRUE;
    OSInsDelay(tcbP);
    err = OSNOERR;
  }      
  else {
    err = OSERR_CT_RUNNING;
  }          
                                             
  OSDecCallDepth();    
  OSLeaveCritical();
  OSMsgRtn("OSStartCycTmr", 
           OSMakeStr("cyclic timer %d started.", OStID(tcbP, OSTASKS)), 
           (OStypeErr) err);
}

#include <salvoscg.h>
#undef __OSSTARTCYCTMR_CYCLIC2_C

#endif /* #if OSENABLE_CYCLIC_TIMERS */



