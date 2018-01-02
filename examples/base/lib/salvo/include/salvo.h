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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvo.h,v $
$Author: aek $
$Revision: 3.109 $
$Date: 2008-06-20 11:37:37-07 $

Primary Salvo header file.

************************************************************/
#ifndef __SALVO_H
#define __SALVO_H


/************************************************************
****                                                     ****
**                                                         **
Version number, e.g. v3.2.3.

**                                                         **
****                                                     ****
************************************************************/
#include <salvover.h>


/************************************************************
****                                                     ****
**                                                         **
Define constants (e.g. OSDONT_START) used throughout.

**                                                         **
****                                                     ****
************************************************************/
#include <salvodef.h>


/************************************************************
****                                                     ****
**                                                         **
Auto-detect compiler in use, set OSCOMPILER and OSTARGET
accordingly.

**                                                         **
****                                                     ****
************************************************************/
#include <salvoadc.h>


/************************************************************
****                                                     ****
**                                                         **
Include appropriate header files based on what type of
build this is (library, library build or source-code build).

If generating a library, no settings should be overridden.
OSMAKE_LIBRARY is expected to be defined outside of source
files, e.g. on the compiler's command-line.

Otherwise include user's/project's salvocfg.h.

salvocfg.h can override (if desired) certain settings in
salvolib.h.

**                                                         **
****                                                     ****
************************************************************/
// Set to FALSE if not defined externally.
#if !defined(OSMAKE_LIBRARY)
#define OSMAKE_LIBRARY                  FALSE
#endif

// If OSMAKE_LIBRARY is TRUE, we're building a library,
//  therefore salvocfg.h is disregarded -- we use
//  salvolib.h in its place.
#if OSMAKE_LIBRARY
#include <salvolib.h>

// If OSMAKE_LIBRARY is FALSE, then we're either doing a
//  library build, or a	source-code build. Either way, we
//  need the project's salvocfg.h for configuration settings.
#else
#include "salvocfg.h"

// OSUSE_LIBRARY symbol always requires definition.
#if !defined(OSUSE_LIBRARY)
#define OSUSE_LIBRARY                   FALSE
#endif

// To get here, we have a user wanting to build an app by
//  linking to libraries. First, verify that user hasn't
//  specified any configuration options that are only for
//  source-code builds. Then pull in the proper Salvo
//  configuration options for the library build.

// Here is also where we can flag what sort of build this
//  is ... Salvo Lite and Salvo Pro source-code builds are
//  unambiguous. But Salvo LE and Pro library builds are
//  indistinguishable from one another ...
#undef OSSALVOLITE
#undef OSSALVOLE
#undef OSSALVOPRO


#if OSUSE_LIBRARY
#include <salvolbo.h>
#include <salvolib.h>

#if   OSLIBRARY_TYPE == OSF
#define OSSALVOLITE						TRUE
#define OSSALVO_STR_TYPE			"Lite"
#elif OSLIBRARY_TYPE == OSL
#define OSSALVOLE							TRUE
#define OSSALVO_STR_TYPE			"LE"
#endif

// Otherwise we're doing a source-code build.
#else /* for source-code builds */

#define OSSALVOPRO						TRUE
#define OSSALVO_STR_TYPE			"Pro"

#endif /* #if !defined(OSUSE_LIBRARY) */
#endif /* #if OSMAKE_LIBRARY */



/************************************************************
****                                                     ****
**                                                         **

**                                                         **
****                                                     ****
************************************************************/
#if   !defined(OSTASKS)
#error salvo.h: salvocfg.h: OSTASKS undefined -- aborting.
#endif

#if !defined(OSEVENT_FLAGS)
#define OSEVENT_FLAGS                   0
#endif

#if !defined(OSEVENTS)
#define OSEVENTS                        0
#endif

#if !defined(OSMESSAGE_QUEUES)
#define OSMESSAGE_QUEUES                0
#endif


/************************************************************
****                                                     ****
**                                                         **
All builds require port-specific headers.

**                                                         **
****                                                     ****
************************************************************/
#include <salvopsh.h>


/************************************************************
****                                                     ****
**                                                         **
General configuration options defined only if not previously
defined.

**                                                         **
****                                                     ****
************************************************************/
#include <salvoscb.h>


/************************************************************
****                                                     ****
**                                                         **
Catch obsolete / invalid configuration parameters.

**                                                         **
****                                                     ****
************************************************************/
#include <salvoocp.h>


/************************************************************
****                                                     ****
**                                                         **
macro to make / concatenate strings -- used in several
portXyz.h files, so define it now.

**                                                         **
****                                                     ****
************************************************************/
#define _OSMkstr(a)                     #a


/************************************************************
****                                                     ****
**                                                         **
Compiler-specific workarounds.

**                                                         **
****                                                     ****
************************************************************/
#include <salvowar.h>


/************************************************************
****                                                     ****
**                                                         **
#defines that lead to conflicts.

**                                                         **
****                                                     ****
************************************************************/
#if ( OSCOMPILER == OSMIX_PC ) && OSUSE_INLINE_OSTIMER
#error salvo.h: In-lining OSTimer() is not permitted with Mix PC Compiler.
#endif


/************************************************************
****                                                     ****
**                                                         **
Special #defines for creating demo versions.

Hard-coded limits for the number of events, message queues
and tasks are used in places like OSCreateTask() and
OSWaitBinSem().

They are enabled when OSSET_LIMITS is TRUE.

Note that these limits do not prevent the creation of large
ecbs, mqcbs and tcbs -- it's just that the code won't allow
access to the ones beyond the set limits.

**                                                         **
****                                                     ****
************************************************************/
#if OSSET_LIMITS
#undef OSENABLE_BOUNDS_CHECKING
#define OSENABLE_BOUNDS_CHECKING        TRUE
#endif


/************************************************************
****                                                     ****
**                                                         **
Special Salvo configurations.

#defines which override other #defines.

**                                                         **
****                                                     ****
************************************************************/
/* overrides for no-priority builds */
#if OSDISABLE_TASK_PRIORITIES
#undef  OSHIGHEST_PRIO
#undef  OSLOWEST_PRIO
#define OSHIGHEST_PRIO                  0
#define OSLOWEST_PRIO                   0
#endif


/* overrides for maximum MISRA-C compatibility */
#if OSENABLE_MISRA_C
#undef  OSENABLE_DEBUGGING_PRINTFS
#define	OSENABLE_DEBUGGING_PRINTFS		FALSE
#endif

/* must have at least one event defined if any of the		*/
/*  event types are enabled.								*/
#if !OSUSE_LIBRARY && ( OSENABLE_BINARY_SEMAPHORES \
   || OSENABLE_SEMAPHORES     || OSENABLE_MESSAGES \
   || OSENABLE_MESSAGE_QUEUES || OSENABLE_EVENT_FLAGS )
 #if !defined(OSEVENTS)
 #error salvo.h: salvocfg.h: OSEVENTS undefined -- aborting.
 #endif
#endif


/* cyclic timers require that timeouts be enabled */
#if OSENABLE_CYCLIC_TIMERS
  #undef OSENABLE_TIMEOUTS
  #define OSENABLE_TIMEOUTS             TRUE
#endif


/* timeouts require that events and delays be enabled */
#if OSENABLE_TIMEOUTS
 #if ( OSBYTES_OF_DELAYS == 0 )
  #undef OSBYTES_OF_DELAYS
  #define OSBYTES_OF_DELAYS             1
 #endif
 #if !OSEVENTS
  #undef OSEVENTS
  #define OSEVENTS                      1
 #endif
#endif


/* must have at least one event flag if they're enabled.    */
/*  An exception exists when using libraries -- you want to */
/*  be able to remove them entirely from your memory map.   */
#if !OSUSE_LIBRARY && OSENABLE_EVENT_FLAGS
 #if !defined(OSEVENT_FLAGS)
 #error salvo.h: salvocfg.h: OSEVENT_FLAGS undefined -- aborting.
 #endif

 #if ( OSEVENT_FLAGS == 0 )
  #undef OSEVENT_FLAGS
  #define OSEVENT_FLAGS                 1
 #endif
#endif


/* must have at least one message queue if they're enabled. */
/*  An exception exists when using libraries -- you want to */
/*  be able to remove them entirely from your memory map.   */
#if !OSUSE_LIBRARY && OSENABLE_MESSAGE_QUEUES
 #if !defined(OSMESSAGE_QUEUES)
 #error salvo.h: salvocfg.h: OSMESSAGE_QUEUES undefined -- aborting.
 #endif

 #if ( OSMESSAGE_QUEUES == 0 )
  #undef OSMESSAGE_QUEUES
  #define OSMESSAGE_QUEUES              1
 #endif
#endif


/* statistics must have counters enabled and > 0 */
#if OSGATHER_STATISTICS
 #if ( OSBYTES_OF_COUNTS == 0 )
 #undef OSBYTES_OF_COUNTS
 #define OSBYTES_OF_COUNTS              1
 #endif
 #if !defined(OSBYTES_OF_COUNTS)
 #define OSBYTES_OF_COUNTS              1
 #endif
#endif


/* message logging is only allowed if logging is enabled */
#if OSLOG_MESSAGES && !OSLOGGING
 #undef OSLOG_MESSAGES
 #define OSLOG_MESSAGES                 OSLOG_NONE
#endif

/* we only support 7 bits of lost tick collecting ... */
#if ( (OSMAX_LOST_TICKS < 1) || (OSMAX_LOST_TICKS > 255) )
#error salvo.h: OSMAX_LOST_TICKS must be > 0 and < 256.
#endif


/* makes sense, esp. for timestamp stuff. */
#if OSBYTES_OF_TICKS && OSBYTES_OF_DELAYS
 #if ( OSBYTES_OF_TICKS < OSBYTES_OF_DELAYS )
 #error salvo.h: OSBYTES_OF_TICKS must be greater than or equal to OSBYTES_OF_DELAYS
 #endif
#endif

#if (OSENABLE_EVENT_FLAGS && OSENABLE_FAST_SIGNALING)
  #error salvo.h: Fast signaling via OSENABLE_FAST_SIGNALING is incompatible with event flags.
  #if defined(OSENABLE_FAST_SIGNALING)
    #undef OSENABLE_FAST_SIGNALING
    #define OSENABLE_FAST_SIGNALING FALSE
  #endif
#endif


/************************************************************
****                                                     ****
**                                                         **
Special SALVO configurations.

meta-#defines based on lower-level #defines.

**                                                         **
****                                                     ****
************************************************************/
#if defined(OSENABLE_COUNTS)
#undef OSENABLE_COUNTS
#endif
#if OSBYTES_OF_COUNTS
#define OSENABLE_COUNTS                 TRUE
#else
#define OSENABLE_COUNTS                 FALSE
#endif

#if defined(OSENABLE_DELAYS)
#undef OSENABLE_DELAYS
#endif
#if OSBYTES_OF_DELAYS
#define OSENABLE_DELAYS                 TRUE
#else
#define OSENABLE_DELAYS                 FALSE
#endif

#if defined(OSENABLE_ERROR_CHECKING)
#undef OSENABLE_ERROR_CHECKING
#endif
#define OSENABLE_ERROR_CHECKING         !OSDISABLE_ERROR_CHECKING

#if OSENABLE_EVENT_TRYING
#undef OSENABLE_EVENT_READING
#define OSENABLE_EVENT_READING          TRUE
#endif

#if defined(OSENABLE_TICKS)
#undef OSENABLE_TICKS
#endif
#if OSBYTES_OF_TICKS
#define OSENABLE_TICKS                  TRUE
#else
#define OSENABLE_TICKS                  FALSE
#endif

#if defined(OSENABLE_PRESCALAR)
#undef OSENABLE_PRESCALAR
#endif
#if ( OSTIMER_PRESCALAR >= 1 )    /* avoid TRUE/FALSE problems with big PS */
#define OSENABLE_PRESCALAR              TRUE
#else
#define OSENABLE_PRESCALAR              FALSE
#endif

#if defined(OSENABLE_EVENTS)
#undef OSENABLE_EVENTS
#endif
#if OSEVENTS
#define OSENABLE_EVENTS                 TRUE
#else
#define OSENABLE_EVENTS                 FALSE
#endif

#if defined(OSENABLE_TASKS)
#undef OSENABLE_TASKS
#endif
#if OSTASKS
#define OSENABLE_TASKS                  TRUE
#else
#define OSENABLE_TASKS                  FALSE
#endif


#if defined(OSARRAY_SIZE_IS_BYTE)
#undef OSARRAY_SIZE_IS_BYTE
#endif
#if defined(OSARRAY_SIZE_IS_WORD)
#undef OSARRAY_SIZE_IS_WORD
#endif
#if OSUSE_ARRAYS
#if ( OSTASKS < 9 )
#define OSARRAY_SIZE_IS_BYTE            TRUE
#else
#define OSARRAY_SIZE_IS_WORD            TRUE
#endif
#endif


#if OSENABLE_EVENTS \
  || ( OSCALL_OSSTARTTASK == OSFROM_FOREGROUND ) \
  || ( OSCALL_OSSTARTTASK == OSFROM_ANYWHERE )
#define OSENABLE_SIGQ                   TRUE
#else
#define OSENABLE_SIGQ                   FALSE
#endif


/************************************************************
****                                                     ****
**                                                         **
Compiler- and target-specific special settings

**                                                         **
****                                                     ****
************************************************************/
#if !defined(OSUSE_WEAK_HOOKS)
#define OSUSE_WEAK_HOOKS                FALSE
#endif


/************************************************************
****                                                     ****
**                                                         **
Provisions for multiple callgraphs (for compilers with
statically allocated parameters, etc., like PICC and Cx51).

**                                                         **
****                                                     ****
************************************************************/
#include <salvomcg.h>


/************************************************************
****                                                     ****
**                                                         **
Settings for OSLOC_XYZ configuration options for targets
with banked memory (e.g. PIC16) or special memory qualifiers
(e.g. 8051).

**                                                         **
****                                                     ****
************************************************************/
#include <salvoloc.h>


/************************************************************
****                                                     ****
**                                                         **
Native typedefs (e.g. OStypeInt8u).

**                                                         **
****                                                     ****
************************************************************/
#include <salvotyp.h>


/************************************************************
****                                                     ****
**                                                         **
Definitions for different context-switching schemes (based
on OSCTXSW_METHOD).

**                                                         **
****                                                     ****
************************************************************/
#include <salvoctx.h>


/************************************************************
****                                                     ****
**                                                         **
Critical section macros (e.g. OSEnterCritical()).

**                                                         **
****                                                     ****
************************************************************/
#include <salvocri.h>


/************************************************************
****                                                     ****
**                                                         **
Macro prototypes (e.g. OS_WaitBinSem()).

**                                                         **
****                                                     ****
************************************************************/
#include <salvompt.h>


/************************************************************
****                                                     ****
**                                                         **
Structure definitions (e.g. tcbs).

**                                                         **
****                                                     ****
************************************************************/
#include <salvostr.h>


/************************************************************
****                                                     ****
**                                                         **
External object (e.g. OSeligQ) declarations. These objects
are located in mem.c.

**                                                         **
****                                                     ****
************************************************************/
#include <salvomem.h>


/************************************************************
****                                                     ****
**                                                         **
General macros (e.g. OSTimedOut()).

**                                                         **
****                                                     ****
************************************************************/
#include <salvomac.h>


/************************************************************
****                                                     ****
**                                                         **
Function Prototypes (e.g. OSWaitEvent()).

**                                                         **
****                                                     ****
************************************************************/
#include <salvofpt.h>


/************************************************************
****                                                     ****
**                                                         **
Abbreviations list.

    address                  addr
    binary                   bin
    change                   change, chg
    check                    chk
    circular                 circ
    create                   create
    configuration            config
    context                  ctx
    current                  c
    cyclic timer             CT
    delay                    delay
    delete                   del
    depth                    depth
    destroy                  destroy
    disable                  dis
    disable interrupt(s)     di
    eligible                 elig
    enable                   en
    enable interrupt(s)      ei
    enter                    enter
    event                    event, ev, e
    event control block      ecb
    error                    err
    from                     fm
    function                 f
    function pointer         FP
    global                   gl
    global type              gltype
    identifier               ID
    include guard            IG
    initialize               init
    insert                   ins
    length                   len
    local                    l
    location                 loc
    maximum                  max
    message                  msg
    message queue            msgQ
    msgQ control block       mqcb
    minimum                  min
    not available            NA
    number                   num
    operating system         OS
    parameter                parm
    pointer                  ptr, p
    pointer to a pointer     pp
    prescalar                PS
    previous                 prev
    priority                 prio
    queue                    Q
    report                   rpt
    reset                    rst
    restore                  rstr
    return                   rtn
    save                     save
    scheduler                sched
    semaphore                sem
    set                      set
    signal                   signal
    stack                    stk
    statistics               stats
    string                   str
    switch                   sw
    synchronize              sync
    task                     task, ta, t
    task control block       tcb
    task function pointer    tFP
    tick                     tick
    timeout                  timeout
    timestamp                TS
    timer                    timer
    toggle                   tgl
    utility                  util
    value                    val
    version                  ver
    wait(ing) (for)          wait, w
    warning                  warn

**                                                         **
****                                                     ****
************************************************************/


/************************************************************
****                                                     ****
**                                                         **
End of salvo.h.

**                                                         **
****                                                     ****
************************************************************/

#endif /* __SALVO_H */
