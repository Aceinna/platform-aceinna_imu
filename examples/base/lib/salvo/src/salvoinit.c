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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoinit.c,v $
$Author: aek $
$Revision: 3.10 $
$Date: 2008-04-27 14:45:37-07 $

OSInit()

************************************************************/

#include <salvo.h>


/************************************************************ 
****                                                     ****
**                                                         **
OSInit()
  
Initialize the static vars.

Note: OSInit() must be called prior to any other OS calls.

**                                                         **
****                                                     ****
************************************************************/
void OSInit( void )
{
  // Clear everything appropriate to 0.          
  #if OSCLEAR_GLOBALS
  
  #if OSENABLE_TASKS
  OSeligQP = (OSgltypeTcbP) 0;
  OScTcbP  = (OSgltypeTcbP) 0;
  #endif
    
  #if OSENABLE_DELAYS || OSENABLE_TIMEOUTS
  OSdelayQP = (OSgltypeTcbP) 0;
  #endif
    
  #if OSENABLE_DELAYS
  OSlostTicks = 0;
  #endif
  
  #if OSGATHER_STATISTICS && OSENABLE_COUNTS
  OSctxSws = 0;
  #endif

  #if OSGATHER_STATISTICS && OSENABLE_COUNTS && OSENABLE_IDLE_COUNTER
  OSidleCtxSws = 0;
  #endif

  #if OSGATHER_STATISTICS && OSENABLE_TIMEOUTS
  OStimeouts = 0;
  #endif
    
  #if OSENABLE_TICKS
  OStimerTicks = 0;
  #endif
   
  #if OSENABLE_EVENTS
  OSsigQinP  = (OSgltypeSigQP) 0;
  OSsigQoutP = (OSgltypeSigQP) 0;
  #endif    
   
  #endif /* #if OSCLEAR_GLOBALS */

  // Initialize non-zero vars.
  #if OSENABLE_PRESCALAR
  OStimerPS = OSTIMER_PRESCALAR;
  #endif

  #if OSLOGGING
  OSerrs = 0;
  OSwarns = 0;
  #endif
        
  #if OSCTXSW_METHOD == OSRTNADDR_IS_VAR
  OSrtnAddr = 0;
  #endif

  #if OSENABLE_STACK_CHECKING    
  OSstkDepth = 0;
  OSmaxStkDepth = 0;
  #endif
    
  // Now that stack checking is initialized it's
  //  OK to start doing it.
  OSIncCallDepth();
      
  OSMsg("OSInit", "done.");
          
  OSDecCallDepth();
}


 
