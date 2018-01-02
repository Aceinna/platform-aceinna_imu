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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoqins.c,v $
$Author: aek $
$Revision: 3.15 $
$Date: 2008-04-27 14:45:32-07 $

Functions to insert element(s) into a priority queue.

************************************************************/

#include <salvo.h>


/************************************************************
****                                                     ****
**                                                         **
OSInsElig(task)

Make the specified task eligible and insert it into the
eligible queue.

Note: This operations happens so many times that it made
sense to turn it into a function.

Note: If OSUSE_OSINSELIG_MACRO is TRUE, then this function is
replaced with an in-line version (without stack
checking, of course).  This diminishes the maximum
required call...return stack depth by 1.

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_OSINSELIG_MACRO

void OSInsElig ( OStypeTcbP tcbP )
{
  OSIncCallDepth();

  // Make task ready to run again.
  tcbP->status.bits.state = OSTCB_TASK_ELIGIBLE;

  // Put it back into the eligible queue.
  #if OSUSE_ARRAYS && OSOPTIMIZE_FOR_SPEED
  OSeligQP |= OScTcbP->prioABits;
  #else
  OSInsPrio(tcbP, &OSeligQP);
  #endif

  OSDecCallDepth();
}

#endif /* #if !OSUSE_OSINSELIG_MACRO */


/************************************************************
****                                                     ****
**                                                         **
Arrays vs. Queues

There are two wholly different ways of running Salvo --
with arrays or priority queues. Priority queues were written
first, then came arrays.

The differences between them are:

Feature                              Arrays      Queues
-------                              ------      ------
number of tasks                      up to 16    up to 256
timing                               constant    variable
shared priorities                    no          yes

**                                                         **
****                                                     ****
************************************************************/

/************************************************************
****                                                     ****
**                                                         **
OSInsPrioA(task)

Insert task into specified priority array.

Task priority dictates where it will go into priority array.

Returns:     OSNOERR


**                                                         **
****                                                     ****
************************************************************/
#if OSUSE_ARRAYS

OStypeErr OSInsPrioA( OStypeTcbP tcbP,
                    OStypePrioAP prioAP )
{
  OSIncCallDepth();

  #if OSOPTIMIZE_FOR_SPEED
  *prioAP |= tcbP->prioABits;
  #else
  *prioAP |= OSBits[tcbP->status.bits.prio];
  #endif

  OSDecCallDepth();

  OSMsgRtn("OSInsPrioQ",
           OSMakeStr("inserted task %d into array %s.",
                     OStID(tcbP, OSTASKS), "unknown"),
           (OStypeErr) OSNOERR);
}

#endif


/************************************************************
****                                                     ****
**                                                         **
OSInsDelayA(task)

Insert task into delay queue. In array mode, the delay queue
is just a simple queue, stuffed at its head in chronological
order, containing tasks that have been delayed.

Returns:     OSNOERR

**                                                         **
****                                                     ****
************************************************************/
#if OSUSE_ARRAYS && OSENABLE_DELAYS

OStypeErr OSInsDelayQ( OStypeTcbP tcbP )
{
  OSIncCallDepth();

  // Simple insertion into head of queue.
  //  Arrays are not differential so there's
  //  no need to adjust delay.
  tcbP->nextDlyTcbP = OSdelayQP;
  OSdelayQP = tcbP;

  OSDecCallDepth();
  OSMsgRtn("OSInsDelayQ",
           "simple queue insertion complete.",
           (OStypeErr) OSNOERR);
}

#endif


/************************************************************
****                                                     ****
**                                                         **
A word on queues:

All queues are priority queues containing tcbs. The queues
are prioritized based on task priority or delay.

The highest-priority element is at the head of the queue,
where elements are usually removed from the queue.  The
lowest-priority element is at the tail of the queue.
Elements may be inserted at any point in the queue.

Each queue pointer is a simple pointer to the head of the
queue.

There are three queues to worry about:

The eligible queue has all the tasks that are ready to run,
sorted in priority order. OSeligQP points to this queue.
There is a single eligible queue.

The delay queue has all the tasks that have a delay
associated with them. These tasks may also be enqueued in
any one of the event queues below. OSdelayQP points to this
queue. There is a single delay queue.

The event queues. There is an event queue for every event,
and the nextTcbP field in the event's control block points
to the head of the corresponding queue. A task may be
simultaneously enqueued in both an event queue and the
delay queue.  Multiple tasks may be in a single event
queue. There is an event queue for every event.

Eligible and delayed tasks are always taken from the head
of the queue -- there's no reason to take them from anywhere
else. Tasks waiting for an event are also taken from the
head of the queue, and the queue pointer is known explicitly
through the OSSignalXyz(ecbP) call that causes a task to
move from an event queue to the eligible queue.

A problem arises in the need to identify which event queue
a task is in.  It can't be derived from traversing the
queue backwards, and the only other method would be to
search all the event queues looking for the task's tcb. So
instead each tcb holds a pointer to the head of its event
queue to facilitate removal.

Note that ecbP is only defined from when a task is made
to wait the event until when it is made eligible again.
Thus this field can be used while the task is running for
other purposes.

If you try to insert an element into a queue that it's
already a member of, you may end up with a circular pointer
reference, which will crash you hard!

History:

Two types of priority queues were supported -- null-
terminated, singly-linked and circular, doubly-linked
queues.

The null-terminated queues differ from the the circular
queues above in that the priority queues are singly-linked
and null terminated.

Note: With this structure it's not possible to
delete the element from the queue by simple pointer
reassignment having only a handle to the element to be
deleted. Instead, the queue must be traversed. But deleting
elements is relatively rare, so it's a reasonable tradeoff.

 headptr
    |
    V
   +---------+  +---------+  +---------+     +---------+
   | nextTcbP|->| nextTcbP|->|  null   |     |  n/a    |
   | (rest)  |  | (rest)  |  | (rest)  |     | (rest)  |
   +---------+  +---------+  +---------+     +---------+


The doubly-linked circular queues is shown below.

 headptr
    |
+---V-------------------------------------+
|  +---------+  +---------+  +---------+  |     +---------+
+->| nextTcbP|->| nextTcbP|->| nextTcbP|--+     |  null   |
+--| prevTcbP|<-| prevTcbP|<-| prevTcbP|<-+     |  null   |
|  | (rest)  |  | (rest)  |  | (rest)  |  |     | (rest)  |
|  +---------+  +---------+  +---------+  |     +---------+
+-----------------------------------------+

null-terminated queues were developed first. It
became apparent that when either a) several tasks are
round-robining or b) tasks need to be removed from the
middles of queues these singly-linked queues required an
unacceptable amount of processing time to implement queue
functions. Circular queues handle these two situations
much better (nearly-constant time instead of linear time as
a function of the number of queue elements), at a cost of
additional tcb pointers and larger codesize.

Circular / doubly-linked queues were abandoned in v2.2
with the coming of arrays, and the maintenance headaches
associated with the conditional calls to circular queue
routines in OSSched() and elsewhere. The functions themselves
worked just fine ...

**                                                         **
****                                                     ****
************************************************************/



/************************************************************
****                                                     ****
**                                                         **
OSInsPrioQ(task, queue)

Insert a task into a priority queue based on task's priority,
or into the delay queue based on its delay.

For priority queues, insert the tcb after all tcbs with equal
or higher priorities.

For the delay queue, insert the tcb after all tcbs with equal
or smaller (remaining) delays.

NOTE: The routines are split among no timeouts/timeouts
enabled. The timeout split is because the delay pointers
share space with the next pointers if timeouts aren't
enabled, but they have their own storage if timeouts
are enabled.

NOTE: Delays are stored incrementally to speed up removal
from the queue. Insertion must take this incremental delay
scheme into account.

Returns: OSNOERR

**                                                         **
****                                                     ****
************************************************************/
/************************************************************
****                                                     ****
**                                                         **
OSInsPrioQ() with or without delays enabled.

NOTE: OSSPEEDUP_QUEUEING has a SUBSTANTIAL (positive) effect
on the while() loop used below to search the delay queue for
the insert point (approx 25% reduction in lines of code).
The downside is that extra RAM is required to hold the delay
in newDelay.

NOTE: priority and totDelay are in a union 'cause they're
exclusive of each other, depending on what type of queue is
being operated on.

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_ARRAYS && !OSENABLE_TIMEOUTS

OStypeErr OSInsPrioQ( OStypeTcbP  tcbP,
                      OStypeTcbPP tcbPP )
{
  OStypeTcbP thisTcbP;
  OStypeTcbP prevTcbP;
  #if !OSDISABLE_TASK_PRIORITIES || OSENABLE_DELAYS
  union {
    #if !OSDISABLE_TASK_PRIORITIES
    OStypePrio prio;    // Priority of new tcb
    #endif

    #if OSENABLE_DELAYS
    OStypeDelay totDelay;    // Tracks cumulative delay
    #endif
  } u;
  #endif

  #if OSENABLE_DELAYS && OSSPEEDUP_QUEUEING
   OStypeDelay newDelay;        // Intended delay of new tcb
  #endif


  OSIncCallDepth();

  // thisTcbP will be used to traverse queue.
   thisTcbP = *tcbPP;

  // Simple insertion into empty queue. This handles
  //  both priority and delay queues.
  if (!thisTcbP) {
    *tcbPP = tcbP;
    tcbP->u2.nextTcbP = (OSgltypeTcbP) 0;

    OSDecCallDepth();
    OSMsgRtn("OSInsPrioQ",
             OSMakeStr("inserted task %d into empty queue %s.",
                       OStID(tcbP, OSTASKS), "unknown"),
             (OStypeErr) OSNOERR);
  }
  else {
    // Setup additional pointers to traverse queue.
    // There's at least one element in the queue.
    prevTcbP = (OSgltypeTcbP) 0;

    // Find insertion point based on task priority ...
    #if OSENABLE_DELAYS
    if (tcbPP != &OSdelayQP) {
    #endif

      // If priorities are enabled, insertion
      //  point could be anywhere ...
      #if !OSDISABLE_TASK_PRIORITIES
      // Hold onto new tcb's priority.
      u.prio = tcbP->status.bits.prio;

      // Move through queue until we've reached the tcb
      //  with lower priority than the one we're inserting
      while ( u.prio >= thisTcbP->status.bits.prio ) {
        prevTcbP = thisTcbP;
        thisTcbP = thisTcbP->u2.nextTcbP;

        if (!thisTcbP) {
            break;
        }
      }

      /* if priorities are disabled, must insert    */
      /*  at end of queue ...                        */
      #else /* #if !OSDISABLE_TASK_PRIORITIES */
      do {
        prevTcbP = thisTcbP;
        thisTcbP = thisTcbP->u2.nextTcbP;
      } while (thisTcbP);
      #endif /* #if !OSDISABLE_TASK_PRIORITIES */


    #if OSENABLE_DELAYS
    }

    // Find insertion point based on delay ...
    else {
      // Needs to be initialized.
      u.totDelay = thisTcbP->dly.delay;

      // Move through queue until new tcb's delay
      //  exceeds that of tcb in delay queue or
      //  we've reached the end of the queue
      // We will insert the tcb between prev and this
      //  when its delay is less than the total delay
      //  in the queue at thisTcbP.
      #if !OSSPEEDUP_QUEUEING
      while (tcbP->dly.delay >= u.totDelay) {

      // By keeping a local copy of the new tcb's
      //  intended delay we can speed things up con-
      //  siderably within the loop below.  The down-
      //  side is that while ROM requirements shrink,
      //  RAM requirements go up, especially apparent
      //  with multi-byte delays.
      #else
      newDelay = tcbP->dly.delay;
      while (newDelay >= u.totDelay) {
      #endif /* #if !OSSPEEDUP_QUEUEING */

        prevTcbP = thisTcbP;
        thisTcbP = thisTcbP->u2.nextTcbP;

        if (!thisTcbP) {
          break;
        }
        else {
          u.totDelay += thisTcbP->dly.delay;
        }
      } /* while () */
                                  
      // Adjust the delay of the element to be
      //   insert to reflect the differential delay
      //   scheme. The element that follows (if any)
      //  must also be adjusted.
      // u.totDelay is used as a placeholder to sim-
      //  plify the code which was too complex for
      //  PICC to evaluate.
      if (thisTcbP) {
        u.totDelay -= thisTcbP->dly.delay;
      }

      tcbP->dly.delay -= u.totDelay;

      if (thisTcbP) {
        u.totDelay = tcbP->dly.delay;
        thisTcbP->dly.delay -= u.totDelay;
      }
    }
    #endif /* #if OSENABLE_DELAYS */

    // Insert new tcb into priority queue -- prev
    //  functions as both a placeholder and an
    //  indicator that we've gone past the first
    //  element in the queue.
    if (!prevTcbP) {
      *tcbPP = tcbP;
    }
    else {
      prevTcbP->u2.nextTcbP = tcbP;
    }

    tcbP->u2.nextTcbP = thisTcbP;

    OSDecCallDepth();
    OSMsgRtn("OSInsPrioQ",
             OSMakeStr("inserted task %d into queue %s.",
                       OStID(tcbP, OSTASKS), "unknown"),
             (OStypeErr) OSNOERR);
  }
}

#endif


/************************************************************
****                                                     ****
**                                                         **
OSInsPrioQ() with timeouts enabled.

If queue is empty, just insert tcb into it as the sole
element.  If queue is not empty, insert the tcb after all
tcbs with equal or higher priorities.

Note: this is _identical_ (postprocessed) to OSInsPrioQ
for the non-timeout case (just doesn't use a union for
priority).

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_ARRAYS && OSENABLE_TIMEOUTS

OStypeErr OSInsPrioQ( OStypeTcbP  tcbP,
                      OStypeTcbPP tcbPP )
{
  OStypeTcbP thisTcbP;
  OStypeTcbP prevTcbP;

  #if !OSDISABLE_TASK_PRIORITIES
  OStypePrio prio;
  #endif


  OSIncCallDepth();

  // hisTcbP will be used to traverse queue.
   thisTcbP = *tcbPP;

  // Simple insertion into empty queue.
  if (thisTcbP == (OSgltypeTcbP) 0) {
    *tcbPP = tcbP;
    tcbP->u2.nextTcbP = (OSgltypeTcbP) 0;

    OSDecCallDepth();
    OSMsgRtn("OSInsPrioQ",
             OSMakeStr("inserted task %d into empty queue %s.",
                       OStID(tcbP, OSTASKS), "unknown"),
             (OStypeErr) OSNOERR);
  }
  else
  {
    // Setup additional pointers to traverse queue.
    // There's at least one element in the queue.
    prevTcbP = (OSgltypeTcbP) 0;

    // If priorities are enabled, insertion point
    //  could be anywhere ...
    #if !OSDISABLE_TASK_PRIORITIES
    // Hold onto new tcb's priority.
    prio = tcbP->status.bits.prio;

    // Move through queue until we've reached the
    //  tcb with lower priority than the one we're
    //  inserting.
    while ((thisTcbP != (OSgltypeTcbP) 0) && (prio >= thisTcbP->status.bits.prio)) {
      prevTcbP = thisTcbP;
      thisTcbP = thisTcbP->u2.nextTcbP;
    }

    // If priorities are disabled, must insert
    //  at end of queue ...
    #else
    do {
      prevTcbP = thisTcbP;
      thisTcbP = thisTcbP->u2.nextTcbP;
    } while (thisTcbP);
    #endif


    // Insert new tcb into priority queue --
    //  prevTcbP functions as both a placeholder
    //  and an indicator that we've gone past the
    //  first element in the queue
    if (prevTcbP == (OSgltypeTcbP) 0) {
      *tcbPP = tcbP;
    }
    else {
      prevTcbP->u2.nextTcbP = tcbP;
    }

    tcbP->u2.nextTcbP = thisTcbP;

    OSDecCallDepth();
    OSMsgRtn("OSInsPrioQ",
             OSMakeStr("inserted task %d into queue %s.",
                       OStID(tcbP, OSTASKS), "unknown"),
             (OStypeErr) OSNOERR);
  }
}

#endif


/************************************************************
****                                                     ****
**                                                         **
OSInsDelayQ(task)

Insert task into delay queue based on its delay.

Returns:     OSNOERR

**                                                         **
****                                                     ****
************************************************************/
/************************************************************
****                                                     ****
**                                                         **
OSInsDelayQ() with timeouts enabled.

If queue is empty, just insert tcb into it as the sole
element.  If queue is not empty, insert the tcb after all
tcbs with equal or smaller (remaining) delays.

Note: Delays are stored incrementally to speed up removal
from the queue. Insertion must take this incremental delay
scheme into account.

Note: OSSPEEDUP_QUEUEING has a SUBSTANTIAL (positive) effect
on the while() loop below used to search for the insert
point (approx 25% reduction in lines of code). The downside
is that extra RAM is required to hold the delay in
new_delay.

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_ARRAYS && OSENABLE_TIMEOUTS

OStypeErr OSInsDelayQ( OStypeTcbP tcbP )
{
  OStypeTcbP thisTcbP;
  OStypeTcbP prevTcbP;
  OStypeDelay totDelay;

  #if OSSPEEDUP_QUEUEING
  OStypeDelay newDelay;        // Intended delay of new tcb
  #endif

  OSIncCallDepth();

  // thisTcbP will be used to traverse queue.
  thisTcbP = OSdelayQP;

  // Simple insertion into empty queue,
  //  no need to adjust delay
  if (thisTcbP == (OSgltypeTcbP) 0) {
    OSdelayQP = tcbP;
    tcbP->nextDlyTcbP = (OSgltypeTcbP) 0;

    OSDecCallDepth();
    OSMsgRtn("OSInsDelayQ",
             "empty queue insertion complete.",
             (OStypeErr) OSNOERR);
  }

  else {
    // Setup additional pointers to traverse queue.
    // There's at least one element in the queue.
    prevTcbP = (OSgltypeTcbP) 0;

    // Needs to be initialized.
    totDelay = thisTcbP->dly.delay;

    // Move through queue until new tcb's delay
    //  exceeds that of tcb in delay queue or
    //  we've reached the end of the queue
    // We will insert the tcb between prev_p and
    //  thisTcbP when its delay is less than the
    //  total delay in the queue at thisTcbP. 
    // By keeping a local copy of the new tcb's
    //  intended delay we can speed things up con-
    //  siderably within the loop below.  The down-
    //  side is that while ROM requirements shrink,
    //  RAM requirements go up, especially apparent
    //  with multi-byte delays.
    #if !OSSPEEDUP_QUEUEING
    while ((thisTcbP != (OSgltypeTcbP) 0) && (tcbP->dly.delay >= totDelay)) {
    #else
    newDelay = tcbP->dly.delay;
    while ((thisTcbP != (OSgltypeTcbP) 0) && (newDelay >= totDelay)) {
    #endif /* #if !OSSPEEDUP_QUEUEING */
      prevTcbP = thisTcbP;
      thisTcbP = thisTcbP->nextDlyTcbP;

      if (thisTcbP != (OSgltypeTcbP) 0) {
        totDelay += thisTcbP->dly.delay;
      }
    }

    // Adjust the delay of the element to be
    //  insert to reflect the differential delay
    //  scheme. The element that follows (if any)
    //  must also be adjusted.
    // u.totDelay is used as a placeholder to simplify
    //  code that was too complex for PICC.
    if (thisTcbP) {
      totDelay -= thisTcbP->dly.delay;
    }

    tcbP->dly.delay -= totDelay;

    if (thisTcbP) {
      totDelay = tcbP->dly.delay;
      thisTcbP->dly.delay -= totDelay;
    }

    // Insert new tcb into priority queue -- prev_p
    //  functions as both a placeholder and an
    //  indicator that we've gone past the first
    //  element in the queue.
    if (prevTcbP == (OSgltypeTcbP) 0) {
      OSdelayQP = tcbP;
    }
    else {
      prevTcbP->nextDlyTcbP = tcbP;
    }

    tcbP->nextDlyTcbP = thisTcbP;

    OSDecCallDepth();
    OSMsgRtn("OSInsPrioQ",
             OSMakeStr("inserted task %d into queue %s.",
                       OStID(tcbP, OSTASKS), "unknown"),
             (OStypeErr) OSNOERR);
  }
}

#endif


