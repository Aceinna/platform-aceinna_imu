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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvoocp.h,v $
$Author: aek $
$Revision: 3.6 $
$Date: 2008-04-27 14:45:21-07 $

Salvo obsolete configuration parameters.

************************************************************/
#ifndef __SALVOOCP_H
#define __SALVOOCP_H

/************************************************************
****                                                     ****
**                                                         **
Unsupported, Invalid and/or obsolete configuration options.

**                                                         **
****                                                     ****
************************************************************/
#if defined(OSBIG_MESSAGE_POINTERS) 
#if ( OSCOMPILER == OSHT_PICC )
#if OSBIG_MESSAGE_POINTERS
#define OSMESSAGE_TYPE                  const
#else
#define OSMESSAGE_TYPE                  void
#endif
#else
#error salvo.h: Unsupported or obsolete configuration option: \
OSBIG_MESSAGE_POINTERS. Use OSMESSAGE_TYPE instead.
#endif
#endif

#if defined(OSCALL_OSCREATEBINSEM)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCALL_OSCREATEBINSEM. Use OSCALL_OSCREATEEVENT instead.
#undef OSCALL_OSCREATEBINSEM
#endif

#if defined(OSCALL_OSCREATEMSG)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCALL_OSCREATEMSG. Use OSCALL_OSCREATEEVENT instead.
#undef OSCALL_OSCREATEMSG
#endif

#if defined(OSCALL_OSCREATEMSGQ)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCALL_OSCREATEMSGQ. Use OSCALL_OSCREATEEVENT instead.
#undef OSCALL_OSCREATEMSGQ
#endif

#if defined(OSCALL_OSCREATESEM)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCALL_OSCREATESEM. Use OSCALL_OSCREATEEVENT instead.
#undef OSCALL_OSCREATESEM
#endif

#if defined(OSCALL_OSSIGNALBINSEM)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCALL_OSSIGNALBINSEM. Use OSCALL_OSSIGNALEVENT instead.
#undef OSCALL_OSSIGNALBINSEM
#endif

#if defined(OSCALL_OSSIGNALMSG)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCALL_OSSIGNALMSG. Use OSCALL_OSSIGNALEVENT instead. 
#undef OSCALL_OSSIGNALMSG
#endif

#if defined(OSCALL_OSSIGNALMSGQ)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCALL_OSSIGNALMSGQ. Use OSCALL_OSSIGNALEVENT instead.
#undef OSCALL_OSSIGNALMSGQ
#endif

#if defined(OSCALL_OSSIGNALSEM)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCALL_OSSIGNALSEM. Use OSCALL_OSSIGNALEVENT instead.
#undef OSCALL_OSSIGNALSEM
#endif

#if defined(OSCLEAR_WATCHDOG_TIMER)
#error salvo.h: Unsupported or obsolete configuration option: \
OSCLEAR_WATCHDOG_TIMER. See watchdog timer hook instead.
#undef OSCLEAR_WATCHDOG_TIMER
#endif

#if defined(OSENABLE_IDLE_TASK)
#error salvo.h: Unsupported or obsolete configuration option: \
OSENABLE_IDLE_TASK.
#undef OSENABLE_IDLE_TASK
#endif

#if defined(OSENABLE_IDLE_TASK_HOOK)
#error salvo.h: Unsupported or obsolete configuration option: \
OSENABLE_IDLE_TASK_HOOK.
#undef OSENABLE_IDLE_TASK_HOOK
#endif

#if defined(OSENABLE_INTERRUPT_HOOKS)
#error salvo.h: Unsupported or obsolete configuration option: \
OSENABLE_INTERRUPT_HOOKS. See interrupt hook instead.
#undef OSENABLE_INTERRUPT_HOOKS
#endif

#if defined(OSMONITOR_KEYWORD_POST) && !defined(OSMONITOR_KEYWORD_POST_PROTO)
#error salvo.h: OSMONITOR_KEYWORD_POST_PROTO must also be defined.
#undef OSMONITOR_KEYWORD_POST
#endif

#if defined(OSMONITOR_KEYWORD_PRE) && !defined(OSMONITOR_KEYWORD_PRE_PROTO)
#error salvo.h: OSMONITOR_KEYWORD_PRE_PROTO must also be defined.
#undef OSMONITOR_KEYWORD_PRE
#endif

#if defined(OSPIC16_GIE_BUG)
#error salvo.h: Unsupported or obsolete configuration option: \
OSPIC16_GIE_BUG.
#undef OSPIC16_GIE_BUG
#endif

#if defined(OSPRESERVE_INTERRUPT_MASK)
#error salvo.h: Unsupported or obsolete configuration option: \
OSPRESERVE_INTERRUPT_MASK. See interrupt hook instead.
#undef OSPRESERVE_INTERRUPT_MASK
#endif

#if defined(OSSUPERTIMER_PRESCALAR)
#error salvo.h: Unsupported or obsolete configuration option: \
OSSUPERTIMER_PRESCALAR.
#undef OSSUPERTIMER_PRESCALAR
#endif

#if defined(OSUSE_ARRAYS)
#if OSUSE_ARRAYS
#error salvo.h: Unsupported or obsolete configuration option: \
OSUSE_ARRAYS. Coming to a later version ...
#undef OSUSE_ARRAYS
#define OSUSE_ARRAYS FALSE
#endif
#endif

#if defined(OSUSE_LOCAL_ECBP)
#error salvo.h: Unsupported or obsolete configuration option: \
OSUSE_LOCAL_ECBP.
#undef OSUSE_LOCAL_ECBP           
#endif

#if defined(OSUSE_LOCAL_TCBP)
#error salvo.h: Unsupported or obsolete configuration option: \
OSUSE_LOCAL_TCBP.
#undef OSUSE_LOCAL_TCBP           
#endif

#if defined(OSUSE_CIRCULAR_QUEUES)
#error salvo.h: Unsupported or obsolete configuration option: \
OSUSE_CIRCULAR_QUEUES.
#undef OSUSE_CIRCULAR_QUEUES
#endif

#if defined(OSUSE_INLINE_OSSCHED)
#if OSUSE_INLINE_OSSCHED
#if ( OSCOMPILER == OSIAR_ICC ) && ( OSTARGET == OSMSP430 )
#error salvo.h: Option not supported with this compiler: OSUSE_INLINE_OSSCHED.
#endif
#endif
#endif

#if defined(OSUSE_INSELIGQ_MACRO)
#error salvo.h: Unsupported or obsolete configuration option: \
OSUSE_INSELIGQ_MACRO. Use OSUSE_INSELIG_MACRO instead.
#undef OSUSE_INSELIGQ_MACRO
#endif

#if defined(OSUSE_SUPERTIMER)
#error salvo.h: Unsupported or obsolete configuration option: \
OSUSE_SUPERTIMER.
#undef OSUSE_SUPERTIMER
#endif

/* The user should not change any of these from their defaults	*/
/*  if the compiler in use doesn't need them.                   */
#if  ( OSCALL_OSCREATEEVENT  != OSFROM_BACKGROUND ) \
  || ( OSCALL_OSGETPRIOTASK  != OSFROM_BACKGROUND ) \
  || ( OSCALL_OSGETSTATETASK != OSFROM_BACKGROUND ) \
  || ( OSCALL_OSMSGQEMPTY    != OSFROM_BACKGROUND ) \
  || ( OSCALL_OSRETURNEVENT  != OSFROM_BACKGROUND ) \
  || ( OSCALL_OSSIGNALEVENT  != OSFROM_BACKGROUND ) \
  || ( OSCALL_OSSTARTTASK    != OSFROM_BACKGROUND )
#if  ( OSCOMPILER != OSHT_PICC )  \
  && ( OSCOMPILER != OSHT_V8C )   \
  && ( OSCOMPILER != OSKEIL_C51 )
#error salvo.h: Unsupported or unnecessary configuration option: OSCALL_OSXYZ.
#endif
#endif


#endif /* __SALVOOCP_H */
