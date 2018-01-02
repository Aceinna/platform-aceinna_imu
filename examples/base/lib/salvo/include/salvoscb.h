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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvoscb.h,v $
$Author: aek $
$Revision: 3.13 $
$Date: 2008-06-20 11:37:38-07 $

Salvo header file for source-code builds.

************************************************************/
#ifndef __SALVOSCB_H
#define __SALVOSCB_H

/************************************************************
****                                                     ****
**                                                         **
General configuration options.

Will take effect only if they're not already defined in
salvocfg.h.

**                                                         **
****                                                     ****
************************************************************/
#if !defined(OSBIG_SEMAPHORES)
#define OSBIG_SEMAPHORES                FALSE
#endif

#if !defined(OSBYTES_OF_COUNTS)
#define OSBYTES_OF_COUNTS               0
#endif

#if !defined(OSBYTES_OF_DELAYS)
#define OSBYTES_OF_DELAYS               0
#endif

#if !defined(OSBYTES_OF_EVENT_FLAGS)
#define OSBYTES_OF_EVENT_FLAGS          1
#endif

#if !defined(OSBYTES_OF_TICKS)
#define OSBYTES_OF_TICKS                0
#endif

#if !defined(OSCALL_OSCREATEEVENT)
#define OSCALL_OSCREATEEVENT            OSFROM_BACKGROUND
#endif

#if !defined(OSCALL_OSGETPRIOTASK)
#define OSCALL_OSGETPRIOTASK            OSFROM_BACKGROUND
#endif

#if !defined(OSCALL_OSGETSTATETASK)
#define OSCALL_OSGETSTATETASK           OSFROM_BACKGROUND
#endif

#if !defined(OSCALL_OSMSGQCOUNT)
#define OSCALL_OSMSGQCOUNT              OSFROM_BACKGROUND
#endif

#if !defined(OSCALL_OSMSGQEMPTY)
#define OSCALL_OSMSGQEMPTY              OSFROM_BACKGROUND
#endif

#if !defined(OSCALL_OSRETURNEVENT)
#define OSCALL_OSRETURNEVENT            OSFROM_BACKGROUND
#endif

#if !defined(OSCALL_OSSIGNALEVENT)
#define OSCALL_OSSIGNALEVENT            OSFROM_BACKGROUND
#endif

#if !defined(OSCALL_OSSTARTTASK)
#define OSCALL_OSSTARTTASK              OSFROM_BACKGROUND
#endif

#if !defined(OSCALLGRAPH_CONTROL_REQUIRED) 
#define OSCALLGRAPH_CONTROL_REQUIRED    FALSE
#endif

#if !defined(OSCLEAR_GLOBALS)
#define OSCLEAR_GLOBALS                 TRUE
#endif

#if !defined(OSCLEAR_UNUSED_POINTERS)
#define OSCLEAR_UNUSED_POINTERS         FALSE
#endif

#if !defined(OSCOMBINE_EVENT_SERVICES)
#define OSCOMBINE_EVENT_SERVICES        FALSE
#endif

#if OSCOMBINE_EVENT_SERVICES == TRUE
#error OSCOMBINE_EVENT_SERVICES Not yet supported in Salvo 4.
#endif


#if !defined(OSCUSTOM_LIBRARY_CONFIG)
#define OSCUSTOM_LIBRARY_CONFIG         0
#endif

#if !defined(OSDISABLE_ERROR_CHECKING)
#define OSDISABLE_ERROR_CHECKING        FALSE
#endif

#if !defined(OSDISABLE_FAST_RESCHEDULING)
#define OSDISABLE_FAST_RESCHEDULING     FALSE
#endif

#if !defined(OSDISABLE_TASK_PRIORITIES)
#define OSDISABLE_TASK_PRIORITIES       FALSE
#endif

#if !defined(OSENABLE_BINARY_SEMAPHORES)
#define OSENABLE_BINARY_SEMAPHORES      FALSE
#endif

#if !defined(OSENABLE_BOUNDS_CHECKING)
#define OSENABLE_BOUNDS_CHECKING        FALSE
#endif

#if !defined(OSENABLE_CYCLIC_TIMERS)
#define OSENABLE_CYCLIC_TIMERS          FALSE
#endif

#if !defined(OSENABLE_EVENT_FLAGS)
#define OSENABLE_EVENT_FLAGS            FALSE
#endif

#if !defined(OSENABLE_EVENT_READING)
#define OSENABLE_EVENT_READING          FALSE
#endif

#if !defined(OSENABLE_EVENT_TRYING)
#define OSENABLE_EVENT_TRYING           FALSE
#endif

#if !defined(OSENABLE_FAST_SIGNALING)
#define OSENABLE_FAST_SIGNALING         FALSE
#endif

#if !defined(OSENABLE_IDLE_COUNTER)
#define OSENABLE_IDLE_COUNTER           FALSE
#endif

#if !defined(OSENABLE_IDLING_HOOK)
#define OSENABLE_IDLING_HOOK            FALSE
#endif

#if !defined(OSENABLE_MESSAGES)
#define OSENABLE_MESSAGES               FALSE
#endif

#if !defined(OSENABLE_MESSAGE_QUEUES)
#define OSENABLE_MESSAGE_QUEUES         FALSE
#endif

#if !defined(OSENABLE_MISRA_C)
#define OSENABLE_MISRA_C                FALSE
#endif

#if !defined(OSENABLE_OSSCHED_DISPATCH_HOOK)
#define OSENABLE_OSSCHED_DISPATCH_HOOK  FALSE
#endif

#if !defined(OSENABLE_OSSCHED_ENTRY_HOOK)
#define OSENABLE_OSSCHED_ENTRY_HOOK     FALSE
#endif

#if !defined(OSENABLE_OSSCHED_RETURN_HOOK)
#define OSENABLE_OSSCHED_RETURN_HOOK    FALSE
#endif

#if !defined(OSENABLE_SEMAPHORES)
#define OSENABLE_SEMAPHORES             FALSE
#endif

#if !defined(OSENABLE_STACK_CHECKING)
#define OSENABLE_STACK_CHECKING         FALSE
#endif

#if !defined(OSENABLE_TCBEXT0)
#define OSENABLE_TCBEXT0                FALSE
#endif

#if !defined(OSENABLE_TCBEXT1)
#define OSENABLE_TCBEXT1                FALSE
#endif

#if !defined(OSENABLE_TCBEXT2)
#define OSENABLE_TCBEXT2                FALSE
#endif

#if !defined(OSENABLE_TCBEXT3)
#define OSENABLE_TCBEXT3                FALSE
#endif

#if !defined(OSENABLE_TCBEXT4)
#define OSENABLE_TCBEXT4                FALSE
#endif

#if !defined(OSENABLE_TCBEXT5)
#define OSENABLE_TCBEXT5                FALSE
#endif

#if !defined(OSENABLE_TIMEOUTS)
#define OSENABLE_TIMEOUTS               FALSE
#endif

#if !defined(OSEVENTS_LIMIT)
#define OSEVENTS_LIMIT                  OSEVENTS
#endif

#if !defined(OSGATHER_STATISTICS)
#define OSGATHER_STATISTICS             FALSE
#endif

#if !defined(OSINTERRUPT_LEVEL)
#define OSINTERRUPT_LEVEL               0
#endif

#if !defined(OSLOGGING)
#define OSLOGGING                       FALSE
#endif

#if !defined(OSLOG_MESSAGES)
#define OSLOG_MESSAGES                  OSLOG_NONE
#endif

#if !defined(OSMAKE_LIBRARY)
#define OSMAKE_LIBRARY                  FALSE
#endif

#if !defined(OSMAX_LOST_TICKS)
#define OSMAX_LOST_TICKS                255
#endif

#if !defined(OSOBJECT_PRAGMAS_REQUIRED)
#define OSOBJECT_PRAGMAS_REQUIRED       FALSE
#endif

#if !defined(OSOPTIMIZE_FOR_SPEED)
#define OSOPTIMIZE_FOR_SPEED            FALSE
#endif

#if !defined(OSRPT_HIDE_INVALID_POINTERS)
#define OSRPT_HIDE_INVALID_POINTERS     TRUE
#endif

#if !defined(OSRPT_SHOW_ONLY_ACTIVE)
#define OSRPT_SHOW_ONLY_ACTIVE          TRUE
#endif

#if !defined(OSRPT_SHOW_TOTAL_DELAY)
#define OSRPT_SHOW_TOTAL_DELAY          TRUE
#endif

#if !defined(OSSET_LIMITS)
#define OSSET_LIMITS                    FALSE
#endif

#if !defined(OSSPEEDUP_QUEUEING)
#define OSSPEEDUP_QUEUEING              FALSE
#endif

#if !defined(OSTARGET)
#define OSTARGET                        OSUNDEF
#endif

#if !defined(OSTASKS)
#define OSTASKS                         0
#endif

#if !defined(OSTASKS_LIMIT)
#define OSTASKS_LIMIT                   OSTASKS
#endif

#if !defined(OSTIMER_PRESCALAR)
#define OSTIMER_PRESCALAR               0
#endif

#if !defined(OSTOOLSET_SUPPORTED)
#define OSTOOLSET_SUPPORTED             FALSE
#endif

#if !defined(OSTYPE_TCBEXT0)
#define OSTYPE_TCBEXT0                  void *
#endif

#if !defined(OSTYPE_TCBEXT1)
#define OSTYPE_TCBEXT1                  void *
#endif

#if !defined(OSTYPE_TCBEXT2)
#define OSTYPE_TCBEXT2                  void *
#endif

#if !defined(OSTYPE_TCBEXT3)
#define OSTYPE_TCBEXT3                  void *
#endif

#if !defined(OSTYPE_TCBEXT4)
#define OSTYPE_TCBEXT4                  void *
#endif

#if !defined(OSTYPE_TCBEXT5)
#define OSTYPE_TCBEXT5                  void *
#endif

#if !defined(OSUSE_ARRAYS)
#define OSUSE_ARRAYS                    FALSE
#endif

#if !defined(OSUSE_EVENT_TYPES)
#define OSUSE_EVENT_TYPES               TRUE
#endif

#if !defined(OSUSE_INLINE_OSSCHED)
#define OSUSE_INLINE_OSSCHED            FALSE
#endif

#if !defined(OSUSE_INLINE_OSTIMER)
#define OSUSE_INLINE_OSTIMER            FALSE
#endif

#if !defined(OSUSE_OSINSELIG_MACRO)
#define OSUSE_OSINSELIG_MACRO           TRUE
#endif

#if !defined(OSUSE_LIBRARY)
#define OSUSE_LIBRARY                   FALSE
#endif




/************************************************************
****                                                     ****
**                                                         **
Compiler- and/or target-specific configuration options.

**                                                         **
****                                                     ****
************************************************************/
#if !defined(OSIAR_PIC18_ATTR_ALL)
#define OSIAR_PIC18_ATTR_ALL                /* i.e. no attribute */
#endif


#if !defined(OSMPLAB_C18_LOC_ALL_NEAR)
#define OSMPLAB_C18_LOC_ALL_NEAR        FALSE
#endif


#if !defined(OSMPLAB_C18_STACK_SIZE)
#define OSMPLAB_C18_STACK_SIZE          512  /* will work for any stack size */
#endif


#if !defined(OSPIC18_INTERRUPT_MASK)
#define OSPIC18_INTERRUPT_MASK          0xC0 /* disable both GIE/GIEH and PEIE/GIEL */
#endif


#endif /* __SALVOSCB_H */
