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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoqdel.c,v $
$Author: aek $
$Revision: 3.13 $
$Date: 2011-06-10 18:08:30-07 $

Functions to delete element(s) from a priority queue.

************************************************************/

#include <salvo.h>


/************************************************************
****                                                     ****
**                                                         **
OSDelPrioA(task)

Delete task from specified priority array.

Task priority dictates where it is in priority array.

Returns:     OSNOERR

**                                                         **
****                                                     ****
************************************************************/
#if OSUSE_ARRAYS

OStypeErr OSDelPrioA( OStypeTcbP tcbP,
                    OStypePrioAP prioAP )
{
  OSIncCallDepth();

  #if OSOPTIMIZE_FOR_SPEED
  *prioAP &= ~(tcbP->prioABits);
  #else
  *prioAP &= ~OSBits[tcbP->status.bits.prio];
  #endif /* #if OSOPTIMIZE_FOR_SPEED */

  OSDecCallDepth();

  OSMsgRtn("OSDelPrioQ",
           "tcb deleted from queue.",
           (OStypeErr) OSNOERR);
}

#endif


/************************************************************
****                                                     ****
**                                                         **
OSDelPrioQ(task, queue)

Delete task from the specified queue. Some versions support
deleting a task from the delay queue -- in those cases,
adjustments to the delay of the following tcb will be made.

Returns:     OSNOERR if tcb was in queue
            OSERR if tcb wasn't in queue

**                                                         **
****                                                     ****
************************************************************/
/************************************************************
****                                                     ****
**                                                         **
OSDelPrioQ() without delays.

NOTE: Deleting an element from a queue is a relatively
rare occurence. Unfortunately, the tcb's queue pointers do
not tell us unambiguously whether or not the tcb is
enqueued in any queue (nextTcbP == 0 could be not enqueued
or at end of queue).  Therefore an exhaustive search of the
queue is required, and it is costly.

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_ARRAYS  && !OSENABLE_DELAYS

OStypeErr OSDelPrioQ( OStypeTcbP  tcbP,
                      OStypeTcbPP tcbPP )
{
  OStypeTcbP thisTcbP;
  OStypeTcbP prevTcbP;
  OStypeTcbP nextTcbP;


  OSIncCallDepth();

  // Initialization. Since tcbP->u2.nextTcbP is
  //  used several times below, we'll help
  //  the compiler by keeping a local copy
  //  for quick access.  
  // We'll start searching at the head of the
  //  queue, and mark that by prev_p's initial
  //  value.
  thisTcbP =            *tcbPP;
  prevTcbP =  (OSgltypeTcbP) 0;
  nextTcbP = tcbP->u2.nextTcbP;

  // Search to end of queue.
  while (thisTcbP) {
    // If pointers match then tcb was in
    //  queue.
    if (thisTcbP == tcbP) {
      // if we're at the head of the
      //  queue then a head ptr adjustment
      //  is required, o/wise it's
      //  a simple advancement.
      if (!prevTcbP) {
        *tcbPP = nextTcbP;
      }
      else {
        prevTcbP->u2.nextTcbP = nextTcbP;
      }

      // Optional, but a good idea.
      #if OSCLEAR_UNUSED_POINTERS
      tcbP->u2.nextTcbP = (OSgltypeTcbP) 0;
      #endif

      // We're done, get out.
      OSDecCallDepth();
      OSMsgRtn("OSDelPrioQ",
               "tcb deleted from queue.",
               (OStypeErr) OSNOERR);

    } /* if */

    // We haven't found the tcb in the
    //  queue yet, so advance towards the 
    //  tail, and keep a pointer to the
    //  previous element.
    prevTcbP = thisTcbP;
    thisTcbP = thisTcbP->u2.nextTcbP;

  } /* while */

  OSDecCallDepth();
  OSMsgRtn("OSDelPrioQ",
           "tcb was not in queue.",
           (OStypeErr) OSERR);
}

#endif



/************************************************************
****                                                     ****
**                                                         **
OSDelPrioQ() with delays enabled.

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_ARRAYS && OSENABLE_DELAYS

OStypeErr OSDelPrioQ( OStypeTcbP  tcbP,
                      OStypeTcbPP tcbPP )
{
  OStypeTcbP thisTcbP;
  OStypeTcbP nextTcbP;
  union {
      OStypeTcbP prevTcbP;
      OStypeDelay tpDelay;    // Used in complex op

  } u;


  OSIncCallDepth();

  // Initialization. Since tcbP->u2.nextTcbP is
  //  used several times below, we'll help
  //  the compiler by keeping a local copy
  //  for quick access.
  // We'll start searching at the head of the
  //  queue, and mark that by prev_p's initial
  //  value.
  thisTcbP = *tcbPP;
  u.prevTcbP = (OSgltypeTcbP) 0;
  nextTcbP = tcbP->u2.nextTcbP;

  // Search to end of queue.
  while (thisTcbP) {
    // If pointers match then tcb was in
    //  queue.
    if (thisTcbP == tcbP) {
      // if we're at the head of the
      //  queue then a head ptr adjustment
      //  is required, o/wise it's
      //  a simple advancement.
      if (!u.prevTcbP) {
        *tcbPP = nextTcbP;
      }
      else {
        u.prevTcbP->u2.nextTcbP = nextTcbP;
      }

      // If we're dealing with the delay
      //  queue, we need to adjust the
      //  delay of the next element, if
      //  it exists.
      if (( tcbPP == &OSdelayQP ) && nextTcbP) {
         u.tpDelay = tcbP->dly.delay;
         nextTcbP->dly.delay += u.tpDelay;
      }


      // Optional, but a good idea.
      #if OSCLEAR_UNUSED_POINTERS
      tcbP->u2.nextTcbP = (OSgltypeTcbP) 0;
      #if !OSENABLE_TICKS
      tcbP->dly.delay = 0;
      #endif
      #endif

      // We're done, get out.
      OSDecCallDepth();
      OSMsgRtn("OSDelPrioQ",
               "tcb deleted from queue.",
               (OStypeErr) OSNOERR);

    } /* if */

    // We haven't found the tcb in the
    //  queue yet, so advance towards the
    //  tail, and keep a pointer to the
    //  previous element.
    u.prevTcbP = thisTcbP;
    thisTcbP = thisTcbP->u2.nextTcbP;

  } /* while */

  OSDecCallDepth();
  OSMsgRtn("OSDelPrioQ",
           "tcb was not in queue.",
           (OStypeErr) OSERR);
}

#endif


/************************************************************
****                                                     ****
**                                                         **
OSDelDelayQ(task)

Delete task from the delay queue.

This fn is called only from OSSignalEvent() and OSSched() --
it's a rare occurence, only when a task times out while
waiting for an event, or when a task that was waiting an
event is signaled before timing out.

Returns:     OSNOERR if tcb was in queue
            OSERR if tcb wasn't in queue

**                                                         **
****                                                     ****
************************************************************/
/************************************************************
****                                                     ****
**                                                         **
OSDelDelayQ() for array mode. Since the array-mode delay
queue is a simple one, this fn simply searches for the tcb
and removes the corresponding task from the delay queue.

**                                                         **
****                                                     ****
************************************************************/
#if OSUSE_ARRAYS && OSENABLE_DELAYS

OStypeErr OSDelDelayQ( OStypeTcbP tcbP )
{
  OStypeTcbP thisTcbP;
  OStypeTcbP prevTcbP;
  OStypeTcbP nextTcbP;


  OSIncCallDepth();

  // Initialization. Since tcbP->u2.nextTcbP is
  //  used several times below, we'll help
  //  the compiler by keeping a local copy
  //  for quick access.
  // We'll start searching at the head of the
  //  queue, and mark that by prev_p's initial
  //  value.
  thisTcbP =         OSdelayQP;
  prevTcbP =  (OSgltypeTcbP) 0;
  nextTcbP = tcbP->nextDlyTcbP;

  // Search to end of queue.
  while (thisTcbP) {
    // If pointers match then tcb was in
    //  queue.
    if (thisTcbP == tcbP) {
      // If we're at the head of the
      //  queue then a head ptr adjustment
      //  is required, o/wise it's
      //  a simple advancement.
      if (!prevTcbP) {
        OSdelayQP = nextTcbP;
      }
      else {
        prevTcbP->nextDlyTcbP = nextTcbP;
      }

      // Optional, but a good idea.
      #if OSCLEAR_UNUSED_POINTERS
      tcbP->nextDlyTcbP = (OSgltypeTcbP) 0;
      #if !OSENABLE_TICKS
      tcbP->dly.delay = 0;
      #endif
      #endif

      // We're done, get out.
      OSDecCallDepth();
      OSMsgRtn("OSDelPrioQ",
               "tcb deleted from queue.",
               (OStypeErr) OSNOERR);

    } /* if */

    // We haven't found the tcb in the
    //  queue yet, so advance towards the
    //  tail, and keep a pointer to the
    //  previous element.
    prevTcbP = thisTcbP;
    thisTcbP = thisTcbP->nextDlyTcbP;

  } /* while */

  OSDecCallDepth();
  OSMsgRtn("OSDelPrioQ",
           "tcb was not in queue.",
           (OStypeErr) OSERR);
}

#endif



/************************************************************
****                                                     ****
**                                                         **
OSDelDelayQ() for queue mode. Deletes
specified task from the delay queue.  Task's nextDlyTcbP
is nulled.

NOTE: Deleting an element from a queue is a relatively
rare occurence. Unfortunately, the tcb's queue pointers do
not tell us unambiguously whether or not the tcb is
enqueued in any queue (nextDlyTcbP == 0 could be not enqueued
or at end of queue).  Therefore an exhaustive search of the
queue is required, and it is costly.

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_ARRAYS && OSENABLE_TIMEOUTS

OStypeErr OSDelDelayQ( OStypeTcbP tcbP )
{
  OStypeTcbP thisTcbP;
  OStypeTcbP nextDlyTcbP;
  union {
      OStypeTcbP prevTcbP;
      OStypeDelay tpDelay;    /* used in complex op     */
  } u;
  
  // When a cyclic timer is removed from the queue, 
  //  e.g. vis OSStopCycTmr(), it must have its delay
  //  adjusted "back" to reflect its total (absolute)
  //  delay, not its differential/relative one in the
  //  delay queue.
  #if OSENABLE_CYCLIC_TIMERS
  OStypeDelay cycTmrDelay = 0;
  #endif
  

  OSIncCallDepth();

  // Initialization. Since tcbP->nextDlyTcbP
  //  is used several times below, we'll help
  //  the compiler by keeping a local copy
  //  for quick access.
  // We'll start searching at the head of the
  //  queue, and mark that by prev_p's initial
  //  value.
  thisTcbP    =         OSdelayQP;
  u.prevTcbP  =  (OSgltypeTcbP) 0;
  nextDlyTcbP = tcbP->nextDlyTcbP;

  // Search to end of queue.
  while (thisTcbP) {
  
    // Keep a running total of the delays
    //  in the queue
    #if OSENABLE_CYCLIC_TIMERS
    cycTmrDelay += thisTcbP->dly.delay;
    #endif
  
    // If pointers match then tcb was in
    //  queue.
    if (thisTcbP == tcbP) {
      // If the element to be removed is
      //  the first in the queue, then a
      //  queue head ptr adjustment is
      //  required.
      // O/wise we just skip over the
      //  element to be removed ...
      if (!u.prevTcbP) {
        OSdelayQP = nextDlyTcbP;
      }
      else {
        u.prevTcbP->nextDlyTcbP = nextDlyTcbP;
      }

      // And if an element followed the
      //  one removed, bump up its delay.
      if (nextDlyTcbP) {
        u.tpDelay = tcbP->dly.delay;
        nextDlyTcbP->dly.delay += u.tpDelay;
      }

      // Optional, but a good idea.
      #if OSCLEAR_UNUSED_POINTERS
      tcbP->nextDlyTcbP = (OSgltypeTcbP) 0;
      #if !OSENABLE_TICKS
      tcbP->dly.delay = 0;
      #endif
      #endif
      
      // Add the delays of all the delay queue elements
      //  ahead of the cyctmr to the cyctmr's delay,
      //  making it absolute again.
      #if OSENABLE_CYCLIC_TIMERS
      tcbP->dly.delay = cycTmrDelay;
      #endif
     
      // We're done, get out.
      OSDecCallDepth();
      OSMsgRtn("OSDelDelayQ",
               "tcb deleted from queue.",
               (OStypeErr) OSNOERR);

      } /* if */

      // We haven't found the tcb in the
      //  queue yet, so advance towards the
      //  tail, and keep a pointer to the
      //  previous element.
      u.prevTcbP = thisTcbP;
      thisTcbP = thisTcbP->nextDlyTcbP;

  } /* while */

  OSDecCallDepth();
  OSMsgRtn("OSDelDelayQ",
           "tcb was not in queue.",
           (OStypeErr) OSERR);
}

#endif



