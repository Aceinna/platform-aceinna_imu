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
places(TM). Copyright (C) 1995-2006 Pumpkin, Inc. and its
Licensor(s). All Rights Reserved.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvolbo.h,v $
$Author: aek $
$Revision: 3.7 $
$Date: 2007-11-09 18:37:58-08 $

Salvo header file for weeding out source-code-build
configuration options mistakenly used when doing a library
build.

************************************************************/
#ifndef __SALVOLBO_H
#define __SALVOLBO_H

#if defined(OSBIG_SEMAPHORES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSBIG_SEMAPHORES.
#endif

#if defined(OSBYTES_OF_COUNTS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSBYTES_OF_COUNTS.
#endif

#if defined(OSBYTES_OF_DELAYS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSBYTES_OF_DELAYS.
#endif

#if defined(OSBYTES_OF_EVENT_FLAGS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSBYTES_OF_EVENT_FLAGS.
#endif

#if defined(OSCALL_OSCREATEEVENT)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCALL_OSCREATEEVENT.
#endif

#if defined(OSCALL_OSGETPRIOTASK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCALL_OSGETPRIOTASK.
#endif

#if defined(OSCALL_OSGETSTATETASK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCALL_OSGETSTATETASK.
#endif

#if defined(OSCALL_OSMSGQCOUNT)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCALL_OSMSGQCOUNT.
#endif

#if defined(OSCALL_OSMSGQEMPTY)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCALL_OSMSGQEMPTY.
#endif

#if defined(OSCALL_OSRETURNEVENT)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCALL_OSRETURNEVENT.
#endif

#if defined(OSCALL_OSSIGNALEVENT)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCALL_OSSIGNALEVENT.
#endif

#if defined(OSCALL_OSSTARTTASK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCALL_OSSTARTTASK.
#endif

#if defined(OSCLEAR_GLOBALS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCLEAR_GLOBALS.
#endif

#if defined(OSCLEAR_UNUSED_POINTERS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCLEAR_UNUSED_POINTERS.
#endif

#if defined(OSCLEAR_WATCHDOG_TIMER)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCLEAR_WATCHDOG_TIMER().
#endif

#if defined(OSCOLLECT_LOST_TICKS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCOLLECT_LOST_TICKS.
#endif

#if defined(OSCOMBINE_EVENT_SERVICES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCOMBINE_EVENT_SERVICES.
#endif

#if defined(OSCTXSW_METHOD)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSCTXSW_METHOD.
#endif

#if defined(OSDISABLE_ERROR_CHECKING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSDISABLE_ERROR_CHECKING.
#endif

#if defined(OSDISABLE_FAST_SCHEDULING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSDISABLE_FAST_SCHEDULING.
#endif

#if defined(OSDISABLE_TASK_PRIORITIES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSDISABLE_TASK_PRIORITIES.
#endif

#if defined(OSENABLE_BINARY_SEMAPHORES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_BINARY_SEMAPHORES.
#endif

#if defined(OSENABLE_BOUNDS_CHECKING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_BOUNDS_CHECKING.
#endif

#if defined(OSENABLE_CYCLIC_TIMERS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_CYCLIC_TIMERS.
#endif

#if defined(OSENABLE_EVENT_FLAGS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_EVENT_FLAGS.
#endif

#if defined(OSENABLE_EVENT_READING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_EVENT_READING.
#endif

#if defined(OSENABLE_EVENT_TRYING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_EVENT_TRYING.
#endif

#if defined(OSENABLE_FAST_SIGNALING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_FAST_SIGNALING.
#endif

#if defined(OSENABLE_IDLE_COUNTER)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_IDLE_COUNTER.
#endif

#if defined(OSENABLE_IDLING_HOOK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_IDLING_HOOK.
#endif

#if defined(OSENABLE_INTERRUPT_HOOKS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_INTERRUPT_HOOKS.
#endif

#if defined(OSENABLE_MESSAGES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_MESSAGES.
#endif

#if defined(OSENABLE_MESSAGE_QUEUES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_MESSAGE_QUEUES.
#endif

#if defined(OSENABLE_OSSCHED_DISPATCH_HOOK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_OSSCHED_DISPATCH_HOOK.
#endif

#if defined(OSENABLE_OSSCHED_ENTRY_HOOK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_OSSCHED_ENTRY_HOOK.
#endif

#if defined(OSENABLE_OSSCHED_RETURN_HOOK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_OSSCHED_RETURN_HOOK.
#endif

#if defined(OSENABLE_SEMAPHORES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_SEMAPHORES.
#endif

#if defined(OSENABLE_STACK_CHECKING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_STACK_CHECKING.
#endif

#if defined(OSENABLE_TCBEXT0)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_TCBEXT0.
#endif

#if defined(OSENABLE_TCBEXT1)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_TCBEXT1.
#endif

#if defined(OSENABLE_TCBEXT2)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_TCBEXT2.
#endif

#if defined(OSENABLE_TCBEXT3)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_TCBEXT3.
#endif

#if defined(OSENABLE_TCBEXT4)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_TCBEXT4.
#endif

#if defined(OSENABLE_TCBEXT5)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_TCBEXT5.
#endif

#if defined(OSENABLE_TIMEOUTS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSENABLE_TIMEOUTS.
#endif

#if defined(OSGATHER_STATISTICS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSGATHER_STATISTICS.
#endif

#if defined(OSINTERRUPT_LEVEL)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSINTERRUPT_LEVEL.
#endif

#if defined(OSLOC_ALL)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_ALL.
#endif

#if defined(OSLOC_COUNT)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_COUNT.
#endif

#if defined(OSLOC_CTCB)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_CTCB.
#endif

#if defined(OSLOC_DEPTH)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_DEPTH.
#endif

#if defined(OSLOC_ECB)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_ECB.
#endif

#if defined(OSLOC_EFCB)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_EFCB.
#endif

#if defined(OSLOC_ERR)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_ERR.
#endif

#if defined(OSLOC_GLSTAT)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_GLSTAT.
#endif

#if defined(OSLOC_LOGMSG)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_LOGMSG.
#endif

#if defined(OSLOC_LOST_TICK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_LOST_TICK.
#endif

#if defined(OSLOC_MQCB)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_MQCB.
#endif

#if defined(OSLOC_PS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_PS.
#endif

#if defined(OSLOC_TCB)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_TCB.
#endif

#if defined(OSLOC_SIGQ)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_SIGQ.
#endif

#if defined(OSLOC_TICK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOC_TICK.
#endif

#if defined(OSLOGGING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOGGING.
#endif

#if defined(OSLOG_MESSAGES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSLOG_MESSAGES.
#endif

#if defined(OSMESSAGE_TYPE)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSMESSAGE_TYPE.
#endif

#if defined(OSMPLAB_C18_LOC_ALL_NEAR)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSMPLAB_C18_LOC_ALL_NEAR.
#endif

#if defined(OSOPTIMIZE_FOR_SPEED)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSOPTIMIZE_FOR_SPEED.
#endif

#if defined(OSPIC18_INTERRUPT_MASK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSPIC18_INTERRUPT_MASK.
#endif

#if defined(OSPRESERVE_INTERRUPT_MASK)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSPRESERVE_INTERRUPT_MASK.
#endif

#if defined(OSRPT_HIDE_INVALID_POINTERS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSRPT_HIDE_INVALID_POINTERS.
#endif

#if defined(OSRPT_SHOW_ONLY_ACTIVE)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSRPT_SHOW_ONLY_ACTIVE.
#endif

#if defined(OSRPT_SHOW_TOTAL_DELAY)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSRPT_SHOW_TOTAL_DELAY.
#endif

#if defined(OSRTNADDR_OFFSET)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSRTNADDR_OFFSET.
#endif

#if defined(OSSCHED_RETURN_LABEL)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSSCHED_RETURN_LABEL().
#endif

#if defined(OSSET_LIMITS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSSET_LIMITS.
#endif

#if defined(OSSPEEDUP_QUEUEING)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSSPEEDUP_QUEUEING.
#endif

#if defined(OSTIMER_PRESCALAR)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSTIMER_PRESCALAR.
#endif

#if defined(OSTYPE_TCBEXT0)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSTYPE_TCBEXT0.
#endif

#if defined(OSTYPE_TCBEXT1)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSTYPE_TCBEXT1.
#endif

#if defined(OSTYPE_TCBEXT2)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSTYPE_TCBEXT2.
#endif

#if defined(OSTYPE_TCBEXT3)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSTYPE_TCBEXT3.
#endif

#if defined(OSTYPE_TCBEXT4)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSTYPE_TCBEXT4.
#endif

#if defined(OSTYPE_TCBEXT5)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSTYPE_TCBEXT5.
#endif

#if defined(OSUSE_CHAR_SIZED_BITFIELDS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSUSE_CHAR_SIZED_BITFIELDS.
#endif

#if defined(OSUSE_EVENT_TYPES)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSUSE_EVENT_TYPES.
#endif

#if defined(OSUSE_INSELIG_MACRO)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSUSE_INSELIG_MACRO.
#endif

#if defined(OSUSE_MEMSET)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSUSE_MEMSET.
#endif

#if defined(OSUSE_VOID_FN_POINTERS)
#error salvolbo.h: salvocfg.h: Illegal configuration option for library build: OSUSE_VOID_FN_POINTERS.
#endif


#endif /* __SALVOLBO_H */
