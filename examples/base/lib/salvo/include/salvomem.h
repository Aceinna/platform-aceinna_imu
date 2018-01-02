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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvomem.h,v $
$Author: aek $
$Revision: 3.5 $
$Date: 2008-05-13 11:06:58-07 $

External declarations for Salvo's global objects (located
in mem.c).

************************************************************/
#ifndef __SALVOMEM_H
#define __SALVOMEM_H

#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************ 
****                                                     ****
**                                                         **
Global Variables:

 OStcbArea[]:   storage for tcbs
 OSeligQP:      tcb ptr to queue of eligible tasks
 OScTcbP:       tcb ptr to current (RUNNING) task
 OSecbArea[]:   storage for ecbs
 OSmqcbArea[]:  storage for mqcbs
 OSdelayQP:     tcb ptr to queue of delayed tasks
 OSstkDepth:    current stack depth used by SALVO services
 OSmaxStkDepth: maximum  "
 OSerrs:        number of errors
 OSwarns:        "        warnings
 OStimeouts:     "        task timeouts
 OSctxSws:       "        context switches
 OSidleCtxSws:   "        idle task context switches
 OStimerTicks:   "        timer ticks
 OStimerPS:     current (dynamic) value of timer prescalar

**                                                         **
****                                                     ****
************************************************************/
#if !defined(__OSMEM_C)

#if OSUSE_ARRAYS
#if OSARRAY_SIZE_IS_BYTE
extern const OStypePrioA OSBits[8];
#else
extern const OStypePrioA OSBits[16]; 
#endif
#endif 

#if OSENABLE_TASKS
#if OSUSE_EXTERN_ARRAY_SIZES
extern OSgltypeTcb OStcbArea[OSTASKS];
#else
extern OSgltypeTcb OStcbArea[];
#endif
#if OSUSE_ARRAYS
extern OSgltypePrioA OSeligQP;
#else
extern OSgltypeTcbP OSeligQP;
#endif 
extern OSgltypeCTcbP OScTcbP; 
#endif


#if OSENABLE_EVENTS
#if OSUSE_EXTERN_ARRAY_SIZES
extern OSgltypeEcb OSecbArea[OSEVENTS];
#else
extern OSgltypeEcb OSecbArea[];
#endif
#endif

#if OSENABLE_SIGQ
extern OSgltypeSigQP OSsigQinP;
extern OSgltypeSigQP OSsigQoutP;
#endif


#if OSENABLE_EVENT_FLAGS && OSEVENT_FLAGS
#if OSUSE_EXTERN_ARRAY_SIZES
extern OSgltypeEfcb OSefcbArea[OSEVENT_FLAGS];
#else
extern OSgltypeEfcb OSefcbArea[];
#endif
#endif


#if OSENABLE_MESSAGE_QUEUES && OSMESSAGE_QUEUES
#if OSUSE_EXTERN_ARRAY_SIZES
extern OSgltypeMqcb OSmqcbArea[OSMESSAGE_QUEUES];
#else
extern OSgltypeMqcb OSmqcbArea[];
#endif
#endif


#if OSENABLE_DELAYS || OSENABLE_TIMEOUTS
extern OSgltypeTcbP OSdelayQP;
#endif


#if OSENABLE_STACK_CHECKING
extern OSgltypeDepth OSstkDepth, OSmaxStkDepth;
#endif


#if OSGATHER_STATISTICS && OSENABLE_COUNTS
extern OSgltypeCount OSctxSws;
#endif


#if OSGATHER_STATISTICS && OSENABLE_COUNTS && OSENABLE_IDLE_COUNTER
extern OSgltypeCount OSidleCtxSws;
#endif 


#if OSGATHER_STATISTICS && OSENABLE_TIMEOUTS
extern OSgltypeErr OStimeouts;
#endif


#if OSLOGGING
extern OSgltypeErr OSerrs, OSwarns;
#endif
 
 
#if OSLOG_MESSAGES > OSLOG_NONE
extern OSgltypeLogMsg OSlogMsg[80];
#endif


#if OSENABLE_TICKS
extern OSgltypeTick OStimerTicks;
#endif


#if OSENABLE_PRESCALAR
extern OSgltypePS OStimerPS;
#endif


#if OSENABLE_DELAYS
extern OSgltypeLostTick OSlostTicks;
#endif


#if ( (OSCTXSW_METHOD == OSVIA_OSDISPATCH) \
  ||  (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WLABEL) \
  ||  (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WPARAM)  )
extern OSgltypeFrameP OSframeP;
#endif


#if OSCTXSW_METHOD == OSRTNADDR_IS_VAR
extern OStypeRtnAddr OSrtnAddr;
#endif


#if ( OSTARGET == OSPIC18 ) && ( OSCOMPILER == OSIAR_ICC )

#define OSLOC_SAVEPIC18INTS __nonbanked 

#if OSPIC18_INTERRUPT_MASK & 0x80
extern OSLOC_SAVEPIC18INTS OStypeInt8u OSsavePIC18GIE;
#endif

#if OSPIC18_INTERRUPT_MASK & 0x40
extern OSLOC_SAVEPIC18INTS OStypeInt8u OSsavePIC18PEIE;
#endif

#undef OSLOC_SAVEPIC18INTS

#endif


#endif /* #if !defined(__OSMEM_C) */


#if !defined(__OSPORTV8_C)
#if OSCOMPILER == OSHT_V8C && OSTARGET == OSVAV8
extern unsigned char OSV8img_CTL_REG;
#endif
#endif /* #if !defined(__OSPORTV8_C) */


#if 0 // nobody references the license --- it's just for embedding into libs
#if !defined(__OSLICENSE_C)
extern const char OSlicense[];
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __SALVOMEM_H */
