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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvotimer.c,v $
$Author: aek $
$Revision: 3.10 $
$Date: 2008-04-27 14:45:25-07 $

System timer function.

************************************************************/

#include <salvo.h>


/************************************************************ 
****                                                     ****
**                                                         **
OSTimer()

For every tick of the system timer, update the systems ticks
appropriately, taking into account the timer prescalar.

Enter with interrupts disabled!

NOTE: this function does not disable or enable interrupts!
    Interrupts must be disabled when it is called. This is
    usually already the case if called from within an ISR, 
    but doing this explicitly (and re-enabling after the 
    call) may be required if OSTimer() is called from 
    elsewhere.

NOTE: OSIncWarns() is used in place of OSWarn() because
    printf() usually needs interrupts to be enabled, and 
    they're not inside of OSTimer().

NOTE: Losing ticks is considered non-fatal, and therefore
      while OSTimer() does return a non-zero error code,
      only OSIncWarns() is called (instead of OSIncErrs()),


**                                                         **
****                                                     ****
************************************************************/
#if OSENABLE_TICKS || OSENABLE_DELAYS                            


#if !OSUSE_INLINE_OSTIMER

#if ( OSCOMPILER == OSKEIL_C51 )
#pragma NOAREGS    // Avoid "using n" interrupt banking issues
#endif

OStypeErr OSTimer ( void )
#endif /* #if !OSUSE_INLINE_OSTIMER */
{
  // We're in, so increment stack depth.
  #if !OSUSE_INLINE_OSTIMER
  OSIncCallDepth();
  #endif

  // If the prescalar is in use, then it needs to be
  //  decremented and checked to see if it needs to be
  //  reloaded. If a reload is required, then we continue
  //  on to the rest of OSTimer()'s work. If not, then
  //  done.
  #if OSENABLE_PRESCALAR
  if (--OStimerPS == 0) {
      OStimerPS = OSTIMER_PRESCALAR;
   #endif
   
    // Increment the system tick counter, post-prescalar
    #if OSENABLE_TICKS 
    OStimerTicks++;
    #endif
    
    // As long as we haven't exceeded the "buffer depth"
    //  (1..N) for ticks, increment them to be used in
    //  OSSched() to process the delay queue. 
    // If we have exceeded the buffer depth, then flag
    //  it as an error.
    #if OSENABLE_DELAYS 
    if (OSlostTicks < OSMAX_LOST_TICKS) {
      OSlostTicks++;
    }
    #if OSENABLE_ERROR_CHECKING
    else {
      OSIncWarns();
      
      #if !OSUSE_INLINE_OSTIMER
      OSDecCallDepth();
      #endif
      
      #if !OSUSE_INLINE_OSTIMER
      return OSERR;
      #else
      return;
      #endif
    }
    #endif /* #if OSENABLE_ERROR_CHECKING */
    #endif /* #if OSENABLE_DELAYS */
      
  #if OSENABLE_PRESCALAR
  } 
  #endif

  // We're done, so decrement stack depth.
  #if !OSUSE_INLINE_OSTIMER
  OSDecCallDepth();
  #endif
  
  #if !OSUSE_INLINE_OSTIMER
  return OSNOERR;
  #else
  return;
  #endif
}

#endif /* #if OSENABLE_TICKS || OSENABLE_DELAYS */
