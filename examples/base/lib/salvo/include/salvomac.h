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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvomac.h,v $
$Author: aek $
$Revision: 3.13 $
$Date: 2008-04-27 14:45:17-07 $

Salvo macros.

************************************************************/
#ifndef __SALVOMAC_H
#define __SALVOMAC_H

/************************************************************
****                                                     ****
**                                                         **
Macros to directly access task and event fields.

Keep in mind that these are costly when the array
index (i.e. tID or eID) is not a compile-time constant.

**                                                         **
****                                                     ****
************************************************************/
#if 1
#define OSECBP(i)                &OSecbArea[(i-1)]
#define OSEFCBP(i)              &OSefcbArea[(i-1)]
#define OSMQCBP(i)              &OSmqcbArea[(i-1)]
#define OSTCBP(i)                &OStcbArea[(i-1)]
#else
#define OSECBP(i)                ((OSecbArea-1)+(i))
#define OSEFCBP(i)              ((OSefcbArea-1)+(i))
#define OSMQCBP(i)              ((OSmqcbArea-1)+(i))
#define OSTCBP(i)                ((OStcbArea-1)+(i))
#endif

#if OSENABLE_TIMEOUTS
#define OSTimedOut()             (OScTcbP->u2.runStatus.state == OSTCB_TASK_TIMED_OUT)
#endif

#if !OSENABLE_EVENTS
#define OSAnyEligibleTasks()     (OSeligQP)
#else
#define OSAnyEligibleTasks()     (OSeligQP || OSsigQoutP)
#endif

#if OSENABLE_DELAYS && OSENABLE_TICKS
#define OSGetTS()                 OSGetTSTask(OScTcbP)
#define OSSetTS(timestamp)        OSSetTSTask(OScTcbP, timestamp)
#define OSSyncTS(interval)        OSSyncTSTask(OScTcbP, interval)
#endif

#define OSTaskStopped(tcbP)       ((tcbP)->status.bits.state == OSTCB_TASK_STOPPED)


/************************************************************
****                                                     ****
**                                                         **
Tcb extension macros.

**                                                         **
****                                                     ****
************************************************************/
#if OSENABLE_TCBEXT0
#define OScTcbExt0        OScTcbP->tcbExt0
#define OStcbExt0(tcbP)    (tcbP)->tcbExt0
#endif

#if OSENABLE_TCBEXT1
#define OScTcbExt1        OScTcbP->tcbExt1
#define OStcbExt1(tcbP)    (tcbP)->tcbExt1
#endif

#if OSENABLE_TCBEXT2
#define OScTcbExt2        OScTcbP->tcbExt2
#define OStcbExt2(tcbP)    (tcbP)->tcbExt2
#endif

#if OSENABLE_TCBEXT3
#define OScTcbExt3        OScTcbP->tcbExt3
#define OStcbExt3(tcbP)    (tcbP)->tcbExt3
#endif

#if OSENABLE_TCBEXT4
#define OScTcbExt4        OScTcbP->tcbExt4
#define OStcbExt4(tcbP)    (tcbP)->tcbExt4
#endif

#if OSENABLE_TCBEXT5
#define OScTcbExt5        OScTcbP->tcbExt5
#define OStcbExt5(tcbP)    (tcbP)->tcbExt5
#endif


/************************************************************
****                                                     ****
**                                                         **
Stack-checking macros.

**                                                         **
****                                                     ****
************************************************************/
#if OSENABLE_STACK_CHECKING
 #define OSIncCallDepth() \
   do { if ( ++OSstkDepth > OSmaxStkDepth ) { OSmaxStkDepth++; } } while (0)
 #define OSDecCallDepth() OSstkDepth--
#else
#define OSIncCallDepth()
#define OSDecCallDepth()
#endif



/************************************************************
****                                                     ****
**                                                         **
OSIncErrs/Warns/Timeouts()

Incrementers for the errors, warnings and timeouts counters.

Suprisingly,

    if ( a < 255 ) a++

is faster and smaller (by one instruction) than

    if ( ++a == 0 ) a--

with PIC C -- that's because the rollover is checked by
putting the result into the w register. With no rollover,
then increment the file register and keep the results there,
too. Of course this only works for 8-bit unsigned counters.

**                                                         **
****                                                     ****
************************************************************/
#if OSLOGGING
 #define OSIncErrs() \
   do { if (OSerrs < 255) { OSerrs++; } } while (0)
 #define OSIncWarns() \
   do { if (OSwarns < 255) { OSwarns++; } } while (0)
#else
 #define OSIncErrs()
 #define OSIncWarns()
#endif

#if OSGATHER_STATISTICS && OSENABLE_TIMEOUTS
 #define OSIncTimeouts() \
   do { if (OStimeouts < 255) { OStimeouts++; } } while (0)
#else
 #define OSIncTimeouts()
#endif


/************************************************************
****                                                     ****
**                                                         **
Debugging / activity logging macros.

Note that a conventional do { } while (0) approach doesn't
work well here because some compilers (e.g. IAR MSP430) do
not like to see any statements or parts of statements
following the return keyword, e.g.

    do { return (OStypeErr) OSNOERR; } while (0);

results in "Unreachable statement(s)" ([20]) and "Non-void
function: explicit "return" <expression>;  expected." [22]
warnings because of its construction.

**                                                         **
****                                                     ****
************************************************************/

#if OSLOGGING

#if OSLOG_MESSAGES < OSLOG_ERRORS
#define OSErr(a, b)                 do { OSIncErrs(); } while (0)
#define OSErrRtn(a, b, c)           do { OSIncErrs(); return (c); } while (0)
#else
#define OSErr(a, b)                 do { OSLogErr((a), (b)); } while (0)
#define OSErrRtn(a, b, c)           do { OSLogErr((a), (b)); return (c); } while (0)
#endif

#if OSLOG_MESSAGES < OSLOG_WARNINGS
#define OSWarn(a, b)                do { OSIncWarns(); } while (0)
#define OSWarnRtn(a, b, c)          do { OSIncWarns(); return (c); } while (0)
#else
#define OSWarn(a, b)                do { OSLogWarn((a), (b)); } while (0)
#define OSWarnRtn(a, b, c)          do { OSLogWarn((a), (b)); return (c); } while (0)
#endif

#if OSLOG_MESSAGES < OSLOG_ALL
#define OSMsg(a, b)
#define OSMsgRtn(a, b, c)           do { return (c); } while (0)
#else
#define OSMsg(a, b)                 do { OSLogMsg((a), (b)); } while (0)
#define OSMsgRtn(a, b, c)           do { OSLogMsg((a), (b)); return (c); } while (0)
#endif

#else /* #if OSLOGGING  */

#define OSErr(a, b)
#define OSErrRtn(a, b, c)           return (c)
#define OSWarn(a, b)
#define OSWarnRtn(a, b, c)          return (c)
#define OSMsg(a, b)
#define OSMsgRtn(a, b, c)           return (c)

#endif /* #if OSLOGGING */

/************************************************************
****                                                     ****
**                                                         **
OSInitTcb/Ecb() normally would use memset(), but banking
issues in the PICs prevent their use there.

**                                                         **
****                                                     ****
************************************************************/
#if OSUSE_MEMSET
#define OSInitTcb(a) memset((void *) (a), '\0', sizeof(OStypeTcb))
#define OSInitEcb(a) memset((void *) (a), '\0', sizeof(OStypeEcb))
#endif


/************************************************************
****                                                     ****
**                                                         **
OSDel|InsDelay|Prio() as a function of array mode or queue
mode.

**                                                         **
****                                                     ****
************************************************************/
#if OSUSE_ARRAYS
#define OSDelDelay(tcbP)             OSDelDelayQ(tcbP)
#define OSDelPrio(tcbP, method)      OSDelPrioA(tcbP, method)
#define OSInsDelay(tcbP)             OSInsDelayQ(tcbP)
#define OSInsPrio(tcbP, method)      OSInsPrioA(tcbP, method)
#else /* #if OSUSE_ARRAYS */
#define OSDelDelay(tcbP)             OSDelDelayQ(tcbP)
#define OSDelPrio(tcbP, method)      OSDelPrioQ(tcbP, method)
#define OSInsDelay(tcbP)             OSInsDelayQ(tcbP)
#define OSInsPrio(tcbP, method)      OSInsPrioQ(tcbP, method)
#endif /* #if OSUSE_ARRAYS */


/************************************************************
****                                                     ****
**                                                         **
OSInsElig() puts the specified task into the eligible
queue. When in use as a macro, it reduces the call ... rtn
depth by 1.

**                                                         **
****                                                     ****
************************************************************/
#if OSUSE_OSINSELIG_MACRO
 #if OSUSE_ARRAYS && OSOPTIMIZE_FOR_SPEED
  #define OSInsElig(tcbP) { \
    tcbP->status.bits.state = OSTCB_TASK_ELIGIBLE; \
    OSeligQP |= tcbP->prioABits; }
 #else
  #define OSInsElig(tcbP) { \
    tcbP->status.bits.state = OSTCB_TASK_ELIGIBLE; \
    OSInsPrio(tcbP, &OSeligQP); }
 #endif
#endif /* #if OSUSE_OSINSELIG_MACRO */


/************************************************************
****                                                     ****
**                                                         **
OSInsSigQ(tcbP, ecbP);

If a task is waiting this event, remove it from the event's
waiting queue and put it in the signaled queue. Used in
OSSignalXyz(). The task's nextTcbP must be cleared here.

**                                                         **
****                                                     ****
************************************************************/
#define OSInsSigQ(a, b) do { \
  if (OSsigQoutP == 0) { \
    OSsigQoutP        =                (a); \
  } \
  else { \
    OSsigQinP->u2.nextTcbP =           (a); \
  } \
  OSsigQinP             =              (a); \
  (b)->tcbP             = (a)->u2.nextTcbP; \
  (a)->u2.nextTcbP      = (OSgltypeTcbP) 0; \
} while (0)


#endif /* __SALVOMAC_H */
