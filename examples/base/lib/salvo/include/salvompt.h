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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvompt.h,v $
$Author: aek $
$Revision: 3.5 $
$Date: 2008-04-27 14:45:21-07 $

Salvo macro prototypes.

************************************************************/
#ifndef __SALVOMPT_H
#define __SALVOMPT_H

/************************************************************ 
****                                                     ****
**                                                         **
OS_Delay/DelayTS/Destroy/Replace/SetPrio/Stop/:

Macros for simple SALVO services that require a context 
switch.

Interrupt control follows the traditional pattern only inside
the called function. This may be incompatible with future
desired functionality, and may need to be brought into line
with what's going on in OSWaitXyz().

**                                                         **
****                                                     ****
************************************************************/
#if   OSENABLE_DELAYS && !OSENABLE_TICKS
#define OS_Delay(delay) { \
    OSDelay(delay); \
    OS_Yield(); } 
#elif OSENABLE_DELAYS && OSENABLE_TICKS
#define OS_Delay(delay) { \
    OSDelay(delay, FALSE); \
    OS_Yield(); } 
#endif


#if OSENABLE_DELAYS && OSENABLE_TICKS
#define OS_DelayTS(delay) { \
    OSDelay(delay, TRUE); \
    OS_Yield(); }
#endif


#define OS_Destroy() { \
    OSDestroy(); \
    OS_Yield(); }
    

#define OS_Replace(tFP, prio) { \
    OSCreateTask(tFP, OScTcbP, prio); \
    OSReturn(); }
    

#define OS_SetPrio(prio)    { \
    OSSetPrio(prio); \
    OS_Yield(); }
   
   
#if   OSENABLE_DELAYS && !OSENABLE_TICKS
#define OS_Stop() { \
    OSDelay(0); \
    OS_Yield(); }
#elif OSENABLE_DELAYS && OSENABLE_TICKS
#define OS_Stop() { \
    OSDelay(0, FALSE); \
    OS_Yield(); }
#else
#define OS_Stop() { \
    OSStop(); \
    OS_Yield(); }
#endif


/************************************************************ 
****                                                     ****
**                                                         **
OSCreateBinSem(ecbP, binSem)
OSSignalBinSem(ecbP)
OS_WaitBinSem(ecbP, timeout)

OSCreateEflag(ecbP, efcbP, eFlag)
OSClrEFlag(ecbP, mask)
OSSetEFlag(ecbP, mask)
OS_WaitEFlag(ecbP, mask, options, timeout)

OSCreateMsg(ecbP, msgP)
OSSignalMsg(ecbP, msgP)
OS_WaitMsg(ecbP, msgP, timeout)

OSCreateMsgQ(ecbP, mqcbP, msgPP, size)
OSSignalMsgQ(ecbP, msgP)
OS_WaitMsgQ(ecbP, msgP, timeout)

OSCreateSem(ecbP, sem)
OSSignalSem(ecbP)
OS_WaitSem(ecbP, timeout)


Definitions for Salvo services implemented as macros that
call functions, and macros for more complex SALVO services 
that require a context switch.

Conditional interrupt control is accomplished inside of the
called function -- if resource is available (i.e. task does
not have to wait), interrupts will be re-enabled and task
will continue without context-switching. O/wise program
flows back to OSSched() with interrupts disabled.

**                                                         **
****                                                     ****
************************************************************/
/* here are the event services for the flattened case.    */
/*  They all have macro front-ends, and all go through    */
/*  event.c                                                */
#if OSCOMBINE_EVENT_SERVICES

/* BINSEM                                                */
#if OSENABLE_BINARY_SEMAPHORES
#define OSCreateBinSem(ecbP, binSem) \
  OSCreateEvent(ecbP, OSEV_BINSEM, binSem)
  
#define OSSignalBinSem(ecbP) \
  OSSignalEvent(ecbP, OSEV_BINSEM)
  
#if !OSENABLE_TIMEOUTS
#define OS_WaitBinSem(ecbP, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_BINSEM) & OSERR_YIELD ) \
        OS_Yield()
#else        
#define OS_WaitBinSem(ecbP, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_BINSEM, timeout) & OSERR_YIELD ) \
        OS_Yield()
#endif  
#endif


/* EFLAG                                                */
#if OSENABLE_EVENT_FLAGS
#define OSCreateEFlag(ecbP, efcbP, eFlag) \
  OSCreateEvent(ecbP, OSEV_EFLAG, efcbP, eFlag)
  
#define OSClrEFlag(ecbP, mask) \
  OSSignalEvent(ecbP, OSEV_EFLAG, mask, OSCLR_EFLAG)
  
#define OSSetEFlag(ecbP, mask) \
  OSSignalEvent(ecbP, OSEV_EFLAG, mask, OSSET_EFLAG)

#if !OSENABLE_TIMEOUTS  
#define OS_WaitEFlag(ecbP, mask, options, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_EFLAG, mask, options) & OSERR_YIELD ) \
        OS_Yield()
#else        
#define OS_WaitEFlag(ecbP, mask, options, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_EFLAG, timeout, mask, options) & OSERR_YIELD ) \
        OS_Yield()
#endif
#endif


/* MSG                                                   */
#if OSENABLE_MESSAGES
#define OSCreateMsg(ecbP, msgP) \
  OSCreateEvent(ecbP, OSEV_MSG, msgP)
  
#define OSSignalMsg(ecbP, msgP) \
  OSSignalEvent(ecbP, OSEV_MSG, msgP)
  
#if !OSENABLE_TIMEOUTS  
#define OS_WaitMsg(ecbP, msgP, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_MSG, msgP) & OSERR_YIELD ) \
        OS_Yield()
#else
#define OS_WaitMsg(ecbP, msgP, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_MSG, timeout, msgP) & OSERR_YIELD ) \
        OS_Yield()
#endif  
#endif


/* MSGQ                                                    */
#if OSENABLE_MESSAGE_QUEUES
#define OSCreateMsgQ(ecbP, mqcbP, msgPP, size) \
  OSCreateEvent(ecbP, OSEV_MSGQ, mqcbP, msgPP, size)
  
#define OSSignalMsgQ(ecbP, msgP) \
  OSSignalEvent(ecbP, OSEV_MSGQ, msgP)
  
#if !OSENABLE_TIMEOUTS  
#define OS_WaitMsgQ(ecbP, msgP, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_MSGQ, msgP) & OSERR_YIELD ) \
        OS_Yield()
#else
#define OS_WaitMsgQ(ecbP, msgP, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_MSGQ, timeout, msgP) & OSERR_YIELD ) \
        OS_Yield()
#endif  
#endif


/* SEM                                                    */
#if OSENABLE_SEMAPHORES
#define OSCreateSem(ecbP, sem) \
  OSCreateEvent(ecbP, OSEV_SEM, sem)
  
#define OSSignalSem(ecbP) \
  OSSignalEvent(ecbP, OSEV_SEM)
  
#if !OSENABLE_TIMEOUTS  
#define OS_WaitSem(ecbP, timeout) \
    while ( OSWaitEvent(ecbP, OSEV_SEM) & OSERR_YIELD ) \
        OS_Yield()
#else
#define OS_WaitSem(ecbP, timeout) \
    while ( OSWaitEvent(ecbP,  OSEV_SEM, timeout) & OSERR_YIELD ) \
        OS_Yield()
#endif  
#endif

/* here are the event services for the non-flattened     */
/*  case. Most of these are direct function calls, but    */
/*  a few have macro front-ends.                        */
#else /* #if OSCOMBINE_EVENT_SERVICES */

#define OSClrEFlag(ecbP, mask) \
  OSSignalEFlag(ecbP, mask, OSCLR_EFLAG)
  
#define OSSetEFlag(ecbP, mask) \
  OSSignalEFlag(ecbP, mask, OSSET_EFLAG)
  
#if !OSENABLE_TIMEOUTS

#define OS_WaitBinSem(ecbP, timeout) \
    while ( OSWaitBinSem(ecbP) & OSERR_YIELD ) \
        OS_Yield()

#define OS_WaitEFlag(ecbP, mask, options, timeout) \
    while ( OSWaitEFlag(ecbP, mask, options) & OSERR_YIELD ) \
        OS_Yield()
        
#define OS_WaitMsg(ecbP, msgP, timeout) \
    while ( OSWaitMsg(ecbP, msgP) & OSERR_YIELD ) \
        OS_Yield()
        
#define OS_WaitMsgQ(ecbP, msgP, timeout) \
    while ( OSWaitMsgQ(ecbP, msgP) & OSERR_YIELD ) \
        OS_Yield()
        
#define OS_WaitSem(ecbP, timeout) \
    while ( OSWaitSem(ecbP) & OSERR_YIELD ) \
        OS_Yield()

#else

#define OS_WaitBinSem(ecbP, timeout) \
    while ( OSWaitBinSem(ecbP, timeout) & OSERR_YIELD ) \
        OS_Yield()

#define OS_WaitEFlag(ecbP, mask, options, timeout) \
    while ( OSWaitEFlag(ecbP, mask, options, timeout) & OSERR_YIELD ) \
        OS_Yield()

#define OS_WaitMsg(ecbP, msgP, timeout) \
    while ( OSWaitMsg(ecbP, msgP, timeout) & OSERR_YIELD ) \
        OS_Yield()

#define OS_WaitMsgQ(ecbP, msgP, timeout) \
    while ( OSWaitMsgQ(ecbP, msgP, timeout) & OSERR_YIELD ) \
        OS_Yield()

#define OS_WaitSem(ecbP, timeout) \
    while ( OSWaitSem(ecbP, timeout) & OSERR_YIELD ) \
        OS_Yield()
                    
#endif /* #if OSENABLE_TIMEOUTS */

#endif /* #if OSCOMBINE_EVENT_SERVICES */


/* and here are the services that are independent of     */
/*  event service flattening.                            */

#if ( OSENABLE_EVENT_READING && !OSENABLE_EVENT_TRYING )
#define OSReadBinSem(ecbP) \
  OSReturnBinSem(ecbP)
  
#define OSReadEFlag(ecbP) \
  OSReturnEFlag(ecbP)
  
#define OSReadSem(ecbP) \
  OSReturnSem(ecbP)
  
#define OSReadMsg(ecbP) \
  OSReturnMsg(ecbP)
  
#define OSReadMsgQ(ecbP) \
  OSReturnMsgQ(ecbP)
#endif


#if ( OSENABLE_EVENT_READING && OSENABLE_EVENT_TRYING )
#define OSReadBinSem(ecbP) \
  OSReturnBinSem(ecbP, FALSE)
  
#define OSReadEFlag(ecbP) \
  OSReturnEFlag(ecbP)
  
#define OSReadSem(ecbP) \
  OSReturnSem(ecbP, FALSE)
  
#define OSReadMsg(ecbP) \
  OSReturnMsg(ecbP, FALSE)
  
#define OSReadMsgQ(ecbP) \
  OSReturnMsgQ(ecbP, FALSE)
  
#define OSTryBinSem(ecbP) \
  OSReturnBinSem(ecbP, TRUE)
  
#define OSTrySem(ecbP) \
  OSReturnSem(ecbP, TRUE)
  
#define OSTryMsg(ecbP) \
  OSReturnMsg(ecbP, TRUE)
  
#define OSTryMsgQ(ecbP) \
  OSReturnMsgQ(ecbP, TRUE)
#endif

#define OSGetPrio() OSGetPrioTask(OScTcbP)
#define OSGetState() OSGetStateTask(OScTcbP)


#endif /* __SALVOMPT_H */
