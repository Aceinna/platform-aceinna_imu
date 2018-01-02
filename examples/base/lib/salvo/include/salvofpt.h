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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvofpt.h,v $
$Author: aek $
$Revision: 3.14 $
$Date: 2008-05-13 11:06:59-07 $

Salvo function prototypes.

************************************************************/
#ifndef __SALVOFPT_H
#define __SALVOFPT_H

#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************
****                                                     ****
**                                                         **
Function prototypes:

Functions that need additional attributes (e.g. reentrant)
have their symbols defined elsewhere (e.g. salvomcg.h).

**                                                         **
****                                                     ****
************************************************************/



/************************************************************
** Event Services                                          **
************************************************************/
#if OSCOMBINE_EVENT_SERVICES

OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr    OSCreateEvent   ( OStypeEcbP      ecbP,
                               OStypeEType     eType,
                               ... )
OSMONITOR_KEYWORD_POST_PROTO
OSCREATEEVENT_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr    OSSignalEvent   ( OStypeEcbP      ecbP,
                               OStypeEType     eType,
                               ... )
OSMONITOR_KEYWORD_POST_PROTO
OSSIGNALEVENT_ATTR ;


#if !OSENABLE_TIMEOUTS
OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr    OSWaitEvent     ( OStypeEcbP      ecbP,
                               OStypeEType     eType,
                               ... )
OSMONITOR_KEYWORD_POST_PROTO ;
#else
OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr    OSWaitEvent     ( OStypeEcbP      ecbP,
                               OStypeEType     eType,
                               OStypeDelay     timeout,
                               ... )
OSMONITOR_KEYWORD_POST_PROTO ;
#endif

#else /* #if OSCOMBINE_EVENT_SERVICES */

OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSCreateBinSem   ( OStypeEcbP      ecbP,
                               OStypeBinSem    binSem )
OSMONITOR_KEYWORD_POST_PROTO
OSCREATEBINSEM_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSCreateEFlag    ( OStypeEcbP      ecbP,
                               OStypeEfcbP     efcbP,
                               OStypeEFlag     value )
OSMONITOR_KEYWORD_POST_PROTO
OSCREATEEFLAG_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSCreateMsg      ( OStypeEcbP      ecbP,
                               OStypeMsgP      msgP )
OSMONITOR_KEYWORD_POST_PROTO
OSCREATEMSG_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSCreateMsgQ     ( OStypeEcbP      ecbP,
                               OStypeMqcbP     mqcbP,
                               OStypeMsgQPP    msgPP,
                               OStypeMsgQSize  msgQSize )
OSMONITOR_KEYWORD_POST_PROTO
OSCREATEMSGQ_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSCreateSem      ( OStypeEcbP      ecbP,
                               OStypeSem       sem )
OSMONITOR_KEYWORD_POST_PROTO
OSCREATESEM_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSSignalBinSem   ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO
OSSIGNALBINSEM_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSSignalEFlag    ( OStypeEcbP      ecbP,
                               OStypeEFlag     mask,
                               OStypeOption    options )
OSMONITOR_KEYWORD_POST_PROTO
OSSIGNALEFLAG_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSSignalMsg      ( OStypeEcbP      ecbP,
                               OStypeMsgP      msgP )
OSMONITOR_KEYWORD_POST_PROTO
OSSIGNALMSG_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSSignalMsgQ     ( OStypeEcbP      ecbP,
                               OStypeMsgP      msgP )
OSMONITOR_KEYWORD_POST_PROTO
OSSIGNALMSGQ_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSSignalSem      ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO
OSSIGNALSEM_ATTR ;


#if !OSENABLE_TIMEOUTS

OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitBinSem     ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitEFlag      ( OStypeEcbP      ecbP,
                               OStypeEFlag     mask,
                               OStypeOption    options )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitEvent      ( OStypeEcbP      ecbP,
                               OStypeBoolean   avail )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitMsg        ( OStypeEcbP      ecbP,
                               OStypeMsgPP     msgPP )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitMsgQ       ( OStypeEcbP      ecbP,
                               OStypeMsgPP     msgPP )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitSem        ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO ;

#else

OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitBinSem     ( OStypeEcbP      ecbP,
                               OStypeDelay     timeout )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitEFlag      ( OStypeEcbP      ecbP,
                               OStypeEFlag     mask,
                               OStypeOption    options,
                               OStypeDelay     timeout )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitEvent      ( OStypeEcbP      ecbP,
                               OStypeBoolean   avail,
                               OStypeDelay     timeout )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitMsg        ( OStypeEcbP      ecbP,
                               OStypeMsgPP     msgPP,
                               OStypeDelay     timeout )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitMsgQ       ( OStypeEcbP      ecbP,
                               OStypeMsgPP     msgPP,
                               OStypeDelay     timeout )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSWaitSem        ( OStypeEcbP      ecbP,
                               OStypeDelay     timeout )
OSMONITOR_KEYWORD_POST_PROTO ;


#endif /* #if !OSENABLE_TIMEOUTS */
#endif /* #if OSCOMBINE_EVENT_SERVICES */

OSMONITOR_KEYWORD_PRE_PROTO
OStypeMsgQSize  OSMsgQCount  ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO
OSMSGQCOUNT_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeMsgQSize  OSMsgQEmpty  ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO
OSMSGQEMPTY_ATTR ;

#if !OSENABLE_EVENT_TRYING

OSMONITOR_KEYWORD_PRE_PROTO
OStypeBinSem OSReturnBinSem  ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNBINSEM_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeEFlag OSReturnEFlag    ( OStypeEcbP      ecbp )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNEFLAG_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeMsgP  OSReturnMsg      ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNMSG_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeMsgP  OSReturnMsgQ     ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNMSGQ_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeSem   OSReturnSem      ( OStypeEcbP      ecbP )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNSEM_ATTR ;

#else /* #if !OSENABLE_EVENT_TRYING  */

OSMONITOR_KEYWORD_PRE_PROTO
OStypeBinSem OSReturnBinSem  ( OStypeEcbP      ecbP,
                               OStypeBoolean   tryBinSem )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNBINSEM_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeEFlag OSReturnEFlag    ( OStypeEcbP      ecbp )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNEFLAG_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeMsgP  OSReturnMsg      ( OStypeEcbP      ecbP,
                               OStypeBoolean   tryMsg )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNMSG_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeMsgP  OSReturnMsgQ     ( OStypeEcbP      ecbP,
                               OStypeBoolean   tryMsgQ )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNMSGQ_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeSem   OSReturnSem      ( OStypeEcbP      ecbP,
                               OStypeBoolean   trySem )
OSMONITOR_KEYWORD_POST_PROTO
OSRETURNSEM_ATTR ;

#endif /* #if !OSENABLE_EVENT_TRYING  */


/***********************************************************
** Task Services                                          **
************************************************************/
OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSCreateTask     ( OStypeTFP       tFP,
                               OStypeTcbP      tcbP,
                               OStypePrio      prio )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
#if OSENABLE_EVENTS
OStypeErr OSDestroyTask      ( OStypeTcbP      tcbP,
                               OStypeID        events )
#else
OStypeErr OSDestroyTask      ( OStypeTcbP      tcbP )
#endif
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeState OSGetStateTask   ( OStypeTcbP      tcbP )
OSMONITOR_KEYWORD_POST_PROTO
OSGETSTATETASK_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypePrio  OSGetPrioTask    ( OStypeTcbP      tcbP )
OSMONITOR_KEYWORD_POST_PROTO
OSGETPRIOTASK_ATTR ;


OStypeErr   OSInitPrioTask   ( OStypeTcbP      tcbP,
                               OStypePrio      prio );

OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSSetPrioTask    ( OStypeTcbP      tcbP,
                               OStypePrio      prio )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSStartTask      ( OStypeTcbP      tcbP )
OSMONITOR_KEYWORD_POST_PROTO
OSSTARTTASK_ATTR ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSStopTask       ( OStypeTcbP      tcbP )
OSMONITOR_KEYWORD_POST_PROTO ;


/************************************************************
** Current Task Services                                   **
************************************************************/
#if !OSENABLE_TICKS
OSMONITOR_KEYWORD_PRE_PROTO
void        OSDelay          ( OStypeDelay     delay )
OSMONITOR_KEYWORD_POST_PROTO ;
#else
OSMONITOR_KEYWORD_PRE_PROTO
void        OSDelay          ( OStypeDelay     delay,
                               OStypeBoolean   useTS )
OSMONITOR_KEYWORD_POST_PROTO ;
#endif


OSMONITOR_KEYWORD_PRE_PROTO
void        OSDestroy        ( void )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
OStypeTS    OSGetTSTask      ( OStypeTcbP      tcbP )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
void OSInterval              ( OStypeDelay     interval )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
void        OSSetPrio        ( OStypePrio      prio )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
void        OSSetTSTask      ( OStypeTcbP      tcbP,
                               OStypeTS        timestamp )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
void        OSStop           ( void )
OSMONITOR_KEYWORD_POST_PROTO ;


OSMONITOR_KEYWORD_PRE_PROTO
void        OSSyncTSTask     ( OStypeTcbP      tcbP,
                               OStypeInterval  interval )
OSMONITOR_KEYWORD_POST_PROTO ;


/************************************************************
** General Services                                        **
************************************************************/
OSMONITOR_KEYWORD_PRE_PROTO
OStypeTick  OSGetTicks       ( void )
OSMONITOR_KEYWORD_POST_PROTO ;

void 		OSClrWDTHook	 ( void );
void 		OSDisableHook	 ( void );
void 		OSEnableHook	 ( void );
void        OSIdlingHook     ( void );
void        OSInit           ( void );
void 		OSRestoreHook	 ( void );
void        OSRpt            ( OStypeID        tasks,
                               OStypeID        events );
void 		OSSaveHook	 	 ( void );


#if (OSCTXSW_METHOD == OSRTNADDR_IS_PARAM)
OSMONITOR_KEYWORD_PRE_PROTO
void        OSSaveRtnAddr    ( OStypeTFP       tFP );
OSMONITOR_KEYWORD_POST_PROTO
#endif

#if !OSUSE_INLINE_OSSCHED
OSMONITOR_KEYWORD_PRE_PROTO
void        OSSched          ( void )
OSMONITOR_KEYWORD_POST_PROTO ;
#endif

#if OSENABLE_OSSCHED_DISPATCH_HOOK
void		OSSchedDispatchHook( void );
#endif

#if OSENABLE_OSSCHED_ENTRY_HOOK
void		OSSchedEntryHook ( void );
#endif

#if OSENABLE_OSSCHED_RETURN_HOOK
void		OSSchedReturnHook ( void );
#endif

OSMONITOR_KEYWORD_PRE_PROTO
void        OSSetTicks       ( OStypeTick      tick )
OSMONITOR_KEYWORD_POST_PROTO ;

#if !OSUSE_INLINE_OSTIMER
OStypeErr   OSTimer          ( void );
#endif


/************************************************************
** Utility Services                                        **
************************************************************/
#if OSLOGGING
OStypeID    OSeID            ( OStypeEcbP      ecbP,
                               OStypeID        events );

char        *OSMakeStr       ( char *          fmt,
                               ...                  );
#endif

#if !(OSUSE_MEMSET)
void        OSInitEcb        ( OStypeEcbP      ecbP );
void        OSInitTcb        ( OStypeTcbP      tcbP );
#endif

void        OSLogErr         ( char *          name,
                               char *          msg  );
void        OSLogMsg         ( char *          name,
                               char *          msg  );
void        OSLogWarn        ( char *          name,
                               char *          msg  );


#if OSUSE_ARRAYS
OStypeTcbP  OSRtnTcbPfmA     ( OStypePrioA     array );
#endif

OStypeErr   OSTaskRunning    ( OStypeTcbP      tcbP );

OStypeErr   OSTaskUsed       ( OStypeTcbP      tcbP );

OStypeID    OStID            ( OStypeTcbP      tcbP,
                               OStypeID        tasks );


/************************************************************
** Queue / Array Services                                  **
************************************************************/
OStypeErr   OSDelDelayQ      ( OStypeTcbP      tcbP );

#if OSUSE_ARRAYS
OStypeErr   OSDelPrioA       ( OStypeTcbP      tcbP,
                               OStypePrioAP    prioAP );
#else
OStypeErr   OSDelPrioQ       ( OStypeTcbP      tcbP,
                               OStypeTcbPP     tcbPP );
#endif

OStypeErr   OSDelTaskQ       ( OStypeTcbP      tcbP,
                               OStypeBoolean   delDelayedTask );
OStypeErr   OSInsDelayQ      ( OStypeTcbP      tcbP );

#if !OSUSE_OSINSELIG_MACRO
void        OSInsElig        ( OStypeTcbP      tcbP );
#endif

#if OSUSE_ARRAYS
OStypeErr   OSInsPrioA       ( OStypeTcbP      tcbP,
                               OStypePrioAP    prioAP );
#else
OStypeErr   OSInsPrioQ       ( OStypeTcbP      tcbP,
                               OStypeTcbPP     tcbPP );
#endif

OStypeErr   OSInsTaskQ       ( OStypeTcbP      tcbP );


/************************************************************
** Cyclic Timer  Services                                  **
************************************************************/
#if OSENABLE_CYCLIC_TIMERS
OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSCreateCycTmr   ( OStypeTFP       tFP,
                               OStypeTcbP      tcbP,
                               OStypeDelay     delay,
                               OStypeDelay     period,
                               OStypeCTMode    mode );
OSMONITOR_KEYWORD_POST_PROTO


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr OSCycTmrRunning    ( OStypeTcbP      tcbP );
OSMONITOR_KEYWORD_POST_PROTO


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSDestroyCycTmr  ( OStypeTcbP      tcbP );
OSMONITOR_KEYWORD_POST_PROTO


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSResetCycTmr    ( OStypeTcbP      tcbP );
OSMONITOR_KEYWORD_POST_PROTO


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr OSSetCycTmrPeriod  ( OStypeTcbP      tcbP,
							   OStypeDelay     period );
OSMONITOR_KEYWORD_POST_PROTO


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSStartCycTmr    ( OStypeTcbP      tcbP );
OSMONITOR_KEYWORD_POST_PROTO


OSMONITOR_KEYWORD_PRE_PROTO
OStypeErr   OSStopCycTmr     ( OStypeTcbP      tcbP );
OSMONITOR_KEYWORD_POST_PROTO
#endif

/************************************************************
** printf()-style debugging                                **
************************************************************/
void        OSPrintEcb       ( OStypeID        tasks,
                               OStypeEcbP      ecbP );
void        OSPrintEcbP      ( OStypeID        events,
                               OStypeEcbP      ecbP,
                               OStypeBoolean   fixedWidth );
void        OSPrintTcb       ( OStypeID        tasks,
                               OStypeID        events,
                               OStypeTcbP      tcbP );
void        OSPrintTcbP      ( OStypeID        tasks,
                               OStypeTcbP      tcbP,
                               OStypeBoolean   fixedWidth );

#ifdef __cplusplus
}
#endif

#endif /* __SALVOFPT_H */
