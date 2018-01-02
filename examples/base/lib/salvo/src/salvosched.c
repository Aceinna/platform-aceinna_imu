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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvosched.c,v $
$Author: aek $
$Revision: 3.56 $
$Date: 2008-04-27 14:45:31-07 $

Functions to run tasks.

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS


/************************************************************
****                                                     ****
**                                                         **
OSSched()

The scheduler has three parts -- first, a part to deal with
events that have been signaled, then a part for tasks with
expired delays and timeouts, and then the part that actually
dispatches the task.

The first part is only active when events are used. A FIFO
queue of signaled events is processed and for each event,
the waiting task is made eligible. Tasks that were waiting
with timeouts are removed from the delay queue and have the
associated fields cleared.

The second part is active only when delays and/or timeouts are
enabled. If one or more delays have expired, those tasks
must be made eligible again. By controlling interrupts here
we can prevent interrupts from being disabled for an unduly
long time.

The third part executes the highest-priority task in the
eligible queue, and upon its return to the scheduler (i.e.
when it context-switches back to the scheduler, from whence
it came) we deal with the task according to its (new) status.

Normally, tasks will return to the scheduler in OSTCB_TASK_ELIGIBLE,
in which case they get re-enqueued into the eligible queue.
But there are some commonly-occuring cases (e.g. when a task
is delayed or delays itself) where the task isn't re-enqueued
automatically into the eligible queue.

NOTE: tasks in the delay queue should be only OSTCB_TASK_WAITING
or OSTCB_TASK_DELAYED.

NOTE: Needs interrupt-control work if OSMONITOR_KEYWORD_PRE
is used ...

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_INLINE_OSSCHED

#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSCHED_SCHED_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
void OSSched( void )
OSMONITOR_KEYWORD_POST
#endif
{
  // NOTE: this is incompatible with in-lining OSSched()!
  #if OSENABLE_DELAYS && OSUSE_ARRAYS
  OStypeTcbP nextTcbP;
  #endif

  // Entering critical section.
  OSEnterCritical();

  // Hit the watchdog timer now, since startup may have
  //  taken a while ...
  OSClrWDTHook();

  // This hook is here so that analysis can be done on
  //  the scheduler. It's not intended for general use.
  #if OSENABLE_OSSCHED_ENTRY_HOOK
  OSSchedEntryHook();
  #endif

  #if !OSUSE_INLINE_OSSCHED
  OSIncCallDepth();
  #endif

  //*******************************************************
  // First we have to process signaled events. Each time
  //  an event is signaled, one or all of the tasks
  //  waiting the event are added to the signaled event
  //  queue, a simple FIFO queue (for speed and behavior).
  // This loop continues until there are no more tasks
  //  in the queue. Tasks can be added to the queue via
  //  interrupts that occur during queue processing.
  // As of v2.3, tasks started via OSStartTask() may also
  //  be in this queue.
  // Tasks that move from the sigQ to the eligQ are
  //  WAITING (event was signaled), WAITING_TO (event was
  //  signaled, timeout is still ticking) or ELIGIBLE
  //  (from OSStartTask()).
  #if OSENABLE_SIGQ
  while (OSsigQoutP) {
    // Handle to highest-priority task waiting the
    //  event at the head of the event queue.
    OScTcbP = OSsigQoutP;

    // Advance event queue out pointer, and null the
    //  tcb's nextTcbP since we just removed it from
    //  the queue.
    OSsigQoutP = OScTcbP->u2.nextTcbP;
    OScTcbP->u2.nextTcbP = (OSgltypeTcbP) 0;

    // Enqueue task into the eligible queue without
    //  changing its state.
    OSInsPrio(OScTcbP, &OSeligQP);

    // Prior to re-enabling interrupts we must update
    //  the inP. If we've reached the end of the queue,
    //  inP must be reset, o/wise leave it alone.
    if (!OSsigQoutP) {
      OSsigQinP = (OSgltypeTcbP) 0;
    }

    // Let interrupts in, as we've successfully
    //  processed one task, and other events may be
    //  signaled. InP and OutP are up-to-date.
    OSLeaveCritical();
    OSEnterCritical();
  }
  #endif /* #if OSENABLE_SIGQ */


  //*******************************************************
  // Only DELAYED, WAITING or SIGNALED tasks, or cyclic
  //  timers,  have any business being in the delayQ.
  // Next we have to process delayed tasks. Either a
  //  counter (when collecting lost ticks) or a flag (a
  //  semaphore, actually) is only sampled here.
  // We'll use OScTcbP as a handle, as it has no real
  //  meaning outside of the dispatcher part of OSSched()
  //  anyway ...
  // Note that since interrupts are disabled here, no
  //  additional protection against interference from
  //  OSTimer() is required.
  #if OSENABLE_DELAYS

  while (OSlostTicks) {
    // Now we're comitted to as many ticks' worth of
    //  delay processing as have occurred since the
    //  last call to OSSched(). Normally, this will be
    //  either 0 or 1. But in those cases where a task
    //  took more than 2 ticks to yield to the 
    //  scheduler, this number will exceed 1. By
    //  processing all the ticks, we "catch up" to 
    //  the current system tick in terms of task
    //  delays, etc.
    OSlostTicks--;

    // We'll use a local handle to speed things up.
    OScTcbP = OSdelayQP;

    // For a non-empty delay queue, we
    //  must decrement the first element's delay. No
    //  further decrements are necessary due to the
    //  differential nature of the delay queue. No more
    //  changes to any tasks' delays will be made after
    //  this point.
    #if !OSUSE_ARRAYS
    if (OScTcbP) {
      --(OScTcbP->dly.delay);
    }
    #endif


    // We'll process every task in the delay queue until
    //  we either reach the end of the queue or find a
    //  task with a non-zero delay.
    #if !OSUSE_ARRAYS
    while (OScTcbP && (OScTcbP->dly.delay == 0)) {
    #else
    while (OScTcbP) {
    #endif

      // tcbP points to a task with an expired delay.
      // Remove task from the delay queue and reset
      //   its delay queue pointers.

      #if !OSUSE_ARRAYS && !OSENABLE_TIMEOUTS
      OSdelayQP = OScTcbP->u2.nextTcbP;

      #if OSCLEAR_UNUSED_POINTERS
      OScTcbP->u2.nextTcbP = (OSgltypeTcbP) 0;
      #endif
      #endif


      #if !OSUSE_ARRAYS && OSENABLE_TIMEOUTS
      OSdelayQP = OScTcbP->nextDlyTcbP;

      #if OSCLEAR_UNUSED_POINTERS
      OScTcbP->nextDlyTcbP = (OSgltypeTcbP) 0;
      #endif
      #endif


      #if OSUSE_ARRAYS
      nextTcbP = OScTcbP->nextDlyTcbP;
      if (!(--(OScTcbP->dly.delay))) {
        OSDelDelay(OScTcbP);
      } // ??
      #endif


      // Additionally, we need to time-stamp the task
      //  for OS_DelayTS()'s use.
      #if OSENABLE_TICKS
      OScTcbP->dly.timestamp = (OStypeTS) OStimerTicks;
      #endif


      // At this point we've removed the task from
      //  delay queue and cleaned up any issues
      //  relating to the task having been delayed.
      //  Now we have to re-enqueue it into the
      //  eligible queue. Do it based on its state,
      //  i.e. DELAYED, WAITING, SIGNALED or cyclic
      //  timer ...

      // Task is DELAYED.
      // This is the simple case for DELAYED tasks.
      //  Since WAITING or SIGNALED tasks can only be
      //  in the delayQ if timeouts are enabled, and
      //  since cyclic timers only exist if timeouts
      //  are enabled, we can do some simplification
      //  here.
      #if !OSENABLE_TIMEOUTS
      OSInsElig(OScTcbP);

      #else
      // Rest of processing only happens if timeouts
      //  are enabled.
      if (OScTcbP->status.bits.state == OSTCB_TASK_DELAYED) {
        OSInsElig(OScTcbP);
      }


      // Tcb belongs to a CYCLIC_TIMER.
      // Run it now with interrupts enabled. The
      //  yielded bit -- which indicates if a cycTmr
      //  is running -- is reset if the cycTmr is a
      //  one-shot and is left alone if the cycTmr is
      //  continuous. One-shots are left hanging, 
      //  continuous ones get re-enqueued. All of them
      //  are executed here from within the scheduler.
      // It's necessary to test the yielded bit
      //  because e.g. a cyclic timer can stop itself.
      // Note that cyclic timers are only active if 
      //  timeouts are enabled.
      #if OSENABLE_CYCLIC_TIMERS
      else if (OScTcbP->status.bits.state == OSTCB_CYCLIC_TIMER)
      {

        OSLeaveCritical();
        OScTcbP->u3.tFP();
        OSEnterCritical();

        if (OScTcbP->status.bits.prio == OSCT_ONE_SHOT) {
          OScTcbP->status.bits.yielded = FALSE;
        }
        
        if (OScTcbP->status.bits.yielded == TRUE) {
           OScTcbP->dly.delay = OScTcbP->u1.period;
           OSInsDelay(OScTcbP);
        }

      } /* else if */
      #endif /* #if OSENABLE_CYCLIC_TIMERS */


      // Task is either WAITING or SIGNALED, and may
      //  have been made to wait with a timeout, in
      //  which case it's also in the delayQ.
      // A WAITING or SIGNALED task that timed out
      //  while waiting failed to obtain an event.
      //  Remove it from whichever queue it's in and
      //  and invoke special handling.
      // Note that this action of deleting a task
      //  from a priority queue is one of only a few
      //  places where ecbP is used, and it's used
      //  here to speed things up considerably.
      // Note that a SIGNALED task will be considered
      //  to have TIMED_OUT even if the event is
      //  signaled before the timeout period expires.
      //  That's because the SIGNALED task will not
      //  have run before the timeout period expires.
      else if (OScTcbP->u1.ecbP != (OSgltypeEcbP) 0) {
          // Tell the world about the timeout.
          OSMsg("OSSched",
                OSMakeStr("task %d timed out waiting for event %d.",
                          OStID(OScTcbP, OSTASKS),
                          OSeID(OScTcbP->u1.ecbP, OSEVENTS)));

          // Log the timeout if we're keeping track
          //  of them ...
          #if OSGATHER_STATISTICS
          OSIncTimeouts();
          #endif

          // A task "moves" from an eventQ to the
          //  sigQ to the eligQ when the event it's
          //  waiting is signaled. At any time, its
          //  timeout can occur. Since we don't know
          //  which queue it's in, we must attempt
          //  removal from all three ...
          // Remove task from whichever prioQ it's
          //  in. Since OSDelPrio() returns an
          //  indication of whether the tcb was in
          //  the prioQ, and since a task can only
          //  be in one prioQ at any time, use this
          //  info to speed things up ...
          // Check-and-remove from eventQ first, then
          //  sigQ, then eligQ. Task is most likely
          //  to still be in the eventQ.
          #if OSOPTIMIZE_FOR_SPEED
          if (OSNOERR == OSDelPrio(OScTcbP, &(OScTcbP->u1.ecbP->tcbP))) {
            ;
          }
          else if (OSNOERR == OSDelPrio(OScTcbP, &OSsigQoutP)) {
            ;
          }
          else {
            OSDelPrio(OScTcbP, &OSeligQP);
          }
          #else
          // Skip the tests (and do a search of all
          //  prioQ's) to save some program memory.
          OSDelPrio(OScTcbP, &(OScTcbP->u1.ecbP->tcbP));
          OSDelPrio(OScTcbP, &OSsigQoutP);
          OSDelPrio(OScTcbP, &OSeligQP);
          #endif /* #if OSOPTIMIZE_FOR_SPEED */


          // Since a timeout has occurred, we must
          //  break the link between the task and the
          //  event it waited ...
          OScTcbP->u1.ecbP = (OSgltypeEcbP) 0;

          // Put task back into the eligQ marked as
          //  TIMED_OUT. We can do this now, since
          //  we're absolutely sure that it's not in
          //  the eligQ.
          OScTcbP->status.bits.state = OSTCB_TASK_TIMED_OUT;
          OSInsPrio(OScTcbP, &OSeligQP);

      } /* if() */

      #endif /* #if !OSENABLE_TIMEOUTS */


    #if OSUSE_ARRAYS
        }
    #endif


    // Update handle for next round ...
    #if OSUSE_ARRAYS
    OScTcbP = nextTcbP;
    #else
    OScTcbP = OSdelayQP;
    #endif


    // Let interrupts in, as we're already committed to
    //  plucking all tasks with expired delays from the
    //  delay queue. This is what v2.1's SuperTimer was
    //  all about ... Note that one such suspension is
    //  good for both configurations (OSCOLLECT_LOST_-
    //  TICKS or not).
    OSLeaveCritical();
    OSEnterCritical();

    } /* while( ) */

  } /* while() or if () */

  #endif /* #if OSENABLE_DELAYS */


  //*******************************************************
  // Now that delayed tasks have been processed,
  //  we're ready to dispatch the most eligible
  //  task ...

  // Array-based approach ...
  // Grab tcb at head of priority queue and remove
  //  the most eligible tcb from the eligible 
  //  queue.
  #if OSUSE_ARRAYS
  OScTcbP = OSRtnTcbPfmA(OSeligQP);
  if (OScTcbP) {

    #if OSOPTIMIZE_FOR_SPEED
    OSeligQP &= ~OScTcbP->prioABits;
    #else
    OSDelPrio(OScTcbP, &OSeligQP);
    #endif


  // Queue-based approach ...
  // Grab (highest-priority, therefore most eligible)
  //  tcb at head of eligQ and remove it. OSeligQP
  //  advances automatically.
  //  Reset pointers if required.
  #else
  OScTcbP = OSeligQP;
  if (OScTcbP) {

    OSeligQP = OSeligQP->u2.nextTcbP;

    #if OSCLEAR_UNUSED_POINTERS
    OScTcbP->u2.nextTcbP = (OSgltypeTcbP) 0;
    #endif

  #endif /* #if OSUSE_ARRAYS */

    // Tasks in the eligible queue are either
    //  ELIGIBLE, WAITING, WAITING_TO or TIMED_OUT.
    //  I.e. they got there through normal operation,
    //  or because they were waiting an event and the
    //  event was signaled, or they waited an event with
    //  a timeout and it was never signaled (timed out).
    //  We'll save the current status in runStatus so
    //  that task status and
    //  priority info is available while running.
    // runStatus gives us a "picture" of the task's
    //  state and priority prior to resuming execution.
    //  It's used to detect, for example, whether the
    //  task timed out before resuming operation.
    //  runStatus is "read-only" to the current task.
    #if OSENABLE_TIMEOUTS
    OScTcbP->u2.runStatus = OScTcbP->status.bits;
    #endif

    // We must mark task's call depth here, while ints
    //  are still disabled.
    OSIncCallDepth();


    // Start / resume task. Get there indirectly through
    //  a function pointer -- for certain distributions,
    //  the call-by-pointer is implementation-dependent.

    // Tasks run with interrupts enabled.
    OSLeaveCritical();

    // This hook is here so that analysis can be done on
    //  the dispatching part of the scheduler. It's not
    //  intended for general use.
    #if OSENABLE_OSSCHED_DISPATCH_HOOK
    OSSchedDispatchHook();
    #endif

    // Dispatch the task according to the method used.
    #if ((OSCTXSW_METHOD == OSVIA_OSDISPATCH) \
      || (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WLABEL) )
    OSDispatch();
    #elif (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WPARAM)
    OSDispatch(OSdispatchParam);
    #else
    OScTcbP->u3.tFP();
    #endif

    // This label is used by some implementations of
    //  the context switcher.
OSSCHED_RETURN_LABEL();
    #if (OSCTXSW_METHOD == OSRTNADDR_IS_VAR)
    OSSaveRtnAddr();
    #endif

    // Task returns to scheduler here in one of the following
    //  states due to a context switch:
    // 
    //   OSTCB_DESTROYED         task ran, destroyed itself
    //   OSTCB_TASK_STOPPED      task ran, stopped itself
    //   OSTCB_TASK_DELAYED      task ran, delayed itself
    //   OSTCB_TASK_WAITING      task ran, is waiting for an
    //                             event (w/optional timeout)
    //   OSTCB_TASK_ELIGIBLE     task ran, will run again 
    //
    // Any eligible tasks must be reenqueued into the eligQ.
    //  Also, we had a context switch, so let's log that.
    // Note: Context switch has redefined the task's "resume 
    //  address."
    // This hook is called after every context switch. It's not
    //  intended for general use. It is _not_called when idling.
    //  Interrupts remain enabled during this hook ...
    #if OSENABLE_OSSCHED_RETURN_HOOK
    OSSchedReturnHook();
    #endif

    // Now we're back into the OS, so gotta protect stuff ...
    OSEnterCritical();

    // Unmark task's call depth.
    OSDecCallDepth();

    // Inc context-switch counter.
    #if OSGATHER_STATISTICS
    OSctxSws++;
    #endif /* #if OSGATHER_STATISTICS */

    if (OScTcbP->status.bits.state == OSTCB_TASK_ELIGIBLE) {
      // Fast rescheduling applies only to queues, and only
      //  affects round-robining (detrimentally). That's
      //  because if TaskA() signals an event upon which
      //  TaskB() is waiting, then TaskA() will be re-enqueued
      //  into the eligible queue _before_ TaskB() (in the
      //  signaled queue) moves to the eligible queue. This
      //  is not the "expected" round-robin behavior.
      // To have more "strict" round-robin behavior, the
      //  current task (TaskA() above) should be added to the 
      //  signaled queue once it returns to the scheduler.
      // Unfortunately this slows down the scheduler - a lot!
      // Note that this is somewhat simpler than OSInsSigQ()
      //  because there's no ecb involved. Note also that
      //  OScTcbP->u2.nextTcbP must be reset -- not optional!
      #if OSDISABLE_FAST_RESCHEDULING && OSENABLE_SIGQ
      if (OSsigQoutP == 0) {
        OSsigQoutP             =         OScTcbP;
      }
      else {
        OSsigQinP->u2.nextTcbP =         OScTcbP;
      }
      OSsigQinP               =          OScTcbP;
      OSsigQinP->u2.nextTcbP  = (OSgltypeTcbP) 0;

      #else
      // O/wise we do fast rescheduling, which puts the
      //  task (when eligible) back into the eligible queue
      //  as quickly as possible.
      #if OSUSE_ARRAYS && OSOPTIMIZE_FOR_SPEED
      OSeligQP |= OScTcbP->prioABits;
      #else
      OSInsPrio(OScTcbP, &OSeligQP);
      #endif
      #endif
    }

  } /* if ( OScTcbP ) */


  // There were no eligible tasks, so just run the idle function
  //  (if enabled).
  else
  {
    // Mark the fact that an idle ctxSw occurred.
    #if OSGATHER_STATISTICS && OSENABLE_COUNTS \
     && OSENABLE_IDLE_COUNTER
    OSidleCtxSws++;
    #endif

    // Run the idling hook with interrupts enabled.
    #if OSENABLE_IDLING_HOOK
    OSLeaveCritical();
    OSIdlingHook();
    OSEnterCritical();
    #endif

  } /* if ( OScTcbP ) ... else */

  // Done -- clean up.
  #if !OSUSE_INLINE_OSSCHED
  OSDecCallDepth();
  #endif

  OSLeaveCritical();
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSSCHED_SCHED_C
#endif

#endif /* #if OSENABLE_TASKS */

