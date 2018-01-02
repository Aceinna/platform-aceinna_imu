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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvodelay.c,v $
$Author: aek $
$Revision: 3.17 $
$Date: 2008-04-27 14:45:41-07 $

OSDelay() -- delay a task.

OS_DelayTS(delay) conditions and actions:

             delay:   20     20     20     20     20

                TS:  261    261    261    261    261

happens at ticks =:  261    265    281    290    301
  
         missed by:    0      4     20     29     40

           diff = :   20     16      0     -9    -20

      OS_Delay(d=):   20     16      0*    11**   20**


* Don't call OS_Delay(0), which would stop the task. Just
keep task eligible and re-enqueue it.

** Re-synch on next TS period (20, 40, 60, ... in this
example).


************************************************************/

#include <salvo.h>

#if OSENABLE_DELAYS


/************************************************************
****                                                     ****
**                                                         **
OSDelay(delay, delayIsInterval)

Delay the current task by the specified number of system
ticks. If the delay specified is an interval, the delay
is calculated relative to the timestamp that was taken when
the task last timed out.

Internal processing varies based on whether ticks are
enabled -- if so, OSDelay() has to detect whether we want
a simple delay or a synchronized delay.

Note: Context switch immediately after OSDelay()!

Note: If the specified delay is 0, the task is stopped.

Note: If delays are enabled, OS_Stop() is implemented
through OSDelay(0) to save some code space and avoid any
problems with the user specifying OSDelay(0). If delays
are not enabled, OSStop() is called directly.

**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSDELAY_DELAY_C
#include <salvomcg.h>
#endif

#if !OSENABLE_TICKS
OSMONITOR_KEYWORD_PRE
void OSDelay( OStypeDelay   delay )
OSMONITOR_KEYWORD_POST
#else
OSMONITOR_KEYWORD_PRE
void OSDelay( OStypeDelay   delay,
              OStypeBoolean useTS )
OSMONITOR_KEYWORD_POST
#endif
{
  #if OSENABLE_TICKS
  OStypeDelay    diff;
  #endif

  OSEnterCritical();
  OSIncCallDepth();

  // Remember, we enter here in the OSTCB_TASK_ELIGIBLE
  //  state.

  // If delay is specified as zero, it means that we
  //  want to stop the current task.
  if (delay == 0) {
    // Make the task STOPPED.
    OScTcbP->status.bits.state = OSTCB_TASK_STOPPED;

    // And tell about it.
    OSDecCallDepth();
    OSLeaveCritical();
    OSMsg("OSDelay", 
          OSMakeStr("task %d stopped.",
                    OStID(OScTcbP, OSTASKS)));
  }

  // A non-zero delay means we want a real delay.
  else {

    #if !OSENABLE_TICKS
    
    // simple case -- just enqueue the task
    //  into the delay queue with the specified
    //  delay, and then fall through to the end
    //  of this function.
    
    // Make the task DELAYED.
    OScTcbP->status.bits.state = OSTCB_TASK_DELAYED;

    // Set the delay field.
    OScTcbP->dly.delay = delay;

    // Put it into the delay queue ...
    #if !OSENABLE_TIMEOUTS && !OSUSE_ARRAYS
    OSInsPrio(OScTcbP, &OSdelayQP);
    #else
    OSInsDelay(OScTcbP);
    #endif

    // ... and fall through to end of fn.

    #else /* OSENABLE_TICKS */

    if (!useTS) {
        // Placeholder (see below).
        diff = delay;
    }
    else {
      // Set the delay field based on the
      //  desired interval and how much time
      //  has elapsed since the task last
      //  timed out. Do it in three separate
      //  steps to avoid type conversion
      //  warnings with this unsigned math.
      diff  =                      delay;
      diff +=     OScTcbP->dly.timestamp;
      diff -= (OStypeDelay) OStimerTicks;

      // (Signed math) comparisons:
      //  no slippage:         diff = delay
      //  some slippage:   0 < diff < delay
      //  maximum slippage:        diff = 0
      //  critical slippage:  diff << delay
      // !!! UNSIGNED MATH IN USE HERE !!!,
      //  therefore diff << delay is seen as
      //  diff > delay.
      // If the slippage is critical,  we'll
      //  add back as many delays as are
      //  required to re-sync us with the
      //  original TS timing.
      // This will result in either:
      //      diff = 0
      //  or                 
      //      0 < diff < delay
      //  but in either case we will have
      //  missed one or more TS intervals.
      // OSlostTSs indicates how many TS
      //  we missed (NOT tied to the task's tcb).
      while (diff > delay) {
        diff += delay;
    
        OSWarn("OSDelay",
               OSMakeStr("task %d overshot specified interval at %d system ticks",
                         OStID(OScTcbP, OSTASKS), OStimerTicks));
      }
    } /* OSENABLE_TICKS */


    // Now we're ready to enqueue the task, and
    //  diff >= 0. If diff = 0, we're at
    //  maximum slippage, and we simply leave
    //  the task as OSTCB_TASK_ELIGIBLE since
    //  there's no time to delay further.
    if (diff > 0) {
      // Make the task DELAYED.
      OScTcbP->status.bits.state = OSTCB_TASK_DELAYED;
    
      // Set the delay field.
      OScTcbP->dly.delay = diff;
    
      // Put it into the delay queue.
      #if !OSENABLE_TIMEOUTS && !OSUSE_ARRAYS
      OSInsPrio(OScTcbP, &OSdelayQP);
      #else
      OSInsDelay(OScTcbP);
      #endif
    }

    #endif /* OSENABLE_TICKS */

    // And tell about it.
    OSDecCallDepth();
    OSLeaveCritical();
    OSMsg("OSDelay", 
          OSMakeStr("task %d delayed for %u ticks.",
                    OStID(OScTcbP, OSTASKS), OScTcbP->dly.delay));
  }
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSDELAY_DELAY_C
#endif

#endif /*  #if OSENABLE_DELAYS */

