/************************************************************ 
Copyright (C) 1995-2008 Pumpkin, Inc. and its
Licensor(s). Freely distributable.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvomcg.h,v $
$Author: aek $
$Revision: 3.17 $
$Date: 2008-04-27 14:45:22-07 $

"mcg": multiple call graphs.

This file has two purposes, and is used to move 
preprocessor directives that would otherwise clutter the
source code to the point of illegibility.

The first purpose is to manage interrupt control for the 
three cases of how certain functions are called -- from 
mainline (background) code only, from interrupt (foreground) 
code only, or from both. 

If the interrupt mask is not preserved, for background-only 
code, interrupts
are disabled and then re-enabled in a conventional manner.
For foreground-only code, interrupts are not controlled
at all -- it's assumed that the functions are called with
interrupts disabled, and any further interrupt control must
be done explicitly by the programmer. For code that calls
a function from both the foreground and the background,
interrupts are re-enabled only if they were enabled when
the function was called -- this prevents unwanted nesting
of interrupts.

If the interrupt mask is preserved, then it's just a 
matter of forcing the functions to be reentrant.

The second purpose is to insert the directives required by 
certain compilers when multiple call graphs exist for
certain Salvo functions.

For example, in HI-TECH PICC -- which does not employ a
parameter stack -- when a function is called from both 
mainline code and from an interrupt, the function must be
declared with #pragma interrupt_level 0 so that the compiler
knows to maintain separate blocks of RAM for the function's
parameters and local variables.

NOTE: Multiple call graphs can occur when OSCALL_OSXYZ is 
set to OSFROM_ANYWHERE and a compiler that does not use
the stack for parameters and auto variables is used.

************************************************************/
#ifndef __SALVOMCG_H
#define __SALVOMCG_H

/************************************************************
****                                                     ****
**                                                         **
Defaults.

Most compilers don't use / need these ...


**                                                         **
****                                                     ****
************************************************************/

#define OSCREATEBINSEM_ATTR
#define OSCREATEEFLAG_ATTR
#define OSCREATEEVENT_ATTR
#define OSCREATEMSG_ATTR
#define OSCREATEMSGQ_ATTR
#define OSCREATESEM_ATTR
#define OSGETPRIOTASK_ATTR
#define OSGETSTATETASK_ATTR
#define OSMSGQCOUNT_ATTR
#define OSMSGQEMPTY_ATTR
#define OSRETURNBINSEM_ATTR
#define OSRETURNEFLAG_ATTR
#define OSRETURNMSG_ATTR
#define OSRETURNMSGQ_ATTR
#define OSRETURNSEM_ATTR
#define OSSIGNALBINSEM_ATTR
#define OSSIGNALEFLAG_ATTR
#define OSSIGNALEVENT_ATTR
#define OSSIGNALMSG_ATTR
#define OSSIGNALMSGQ_ATTR
#define OSSIGNALSEM_ATTR
#define OSSTARTTASK_ATTR

/************************************************************
****                                                     ****
**                                                         **
Keil C51 complier.

If any service needs to be callable from anywhere, we
preserve the interrupt mask.

If any service in a group is to be callable from anywhere,
then all the services in the group are made reentrant.
Function prototypes must also include the reentrant 
attribute! See salvo.h.

**                                                         **
****                                                     ****
************************************************************/
#if ( OSCOMPILER == OSKEIL_C51 )

#if ( OSCALL_OSCREATEEVENT == OSFROM_ANYWHERE )
    #undef  OSPRESERVE_INTERRUPT_MASK
    #define OSPRESERVE_INTERRUPT_MASK   TRUE 
    #undef  OSCREATEBINSEM_ATTR            
    #define OSCREATEBINSEM_ATTR         reentrant
    #undef  OSCREATEEFLAG_ATTR            
    #define OSCREATEEFLAG_ATTR          reentrant
    #undef  OSCREATEEVENT_ATTR            
    #define OSCREATEEVENT_ATTR          reentrant
    #undef  OSCREATEMSG_ATTR            
    #define OSCREATEMSG_ATTR            reentrant
    #undef  OSCREATEMSGQ_ATTR            
    #define OSCREATEMSGQ_ATTR           reentrant
    #undef  OSCREATESEM_ATTR            
    #define OSCREATESEM_ATTR            reentrant
#endif    
        
#if ( OSCALL_OSGETPRIOTASK == OSFROM_ANYWHERE )
    #undef  OSPRESERVE_INTERRUPT_MASK
    #define OSPRESERVE_INTERRUPT_MASK   TRUE 
    #undef  OSGETPRIOTASK_ATTR        
    #define OSGETPRIOTASK_ATTR          reentrant
#endif    
        
        
#if ( OSCALL_OSGETSTATETASK == OSFROM_ANYWHERE )
    #undef  OSPRESERVE_INTERRUPT_MASK
    #define OSPRESERVE_INTERRUPT_MASK   TRUE 
    #undef  OSGETSTATETASK_ATTR        
    #define OSGETSTATETASK_ATTR         reentrant
#endif    
        
        
#if ( OSCALL_OSMSGQCOUNT == OSFROM_ANYWHERE )
    #undef  OSPRESERVE_INTERRUPT_MASK
    #define OSPRESERVE_INTERRUPT_MASK   TRUE 
    #undef  OSMSGQCOUNT_ATTR        
    #define OSMSGQCOUNT_ATTR            reentrant
#endif    
        
        
#if ( OSCALL_OSMSGQEMPTY == OSFROM_ANYWHERE )
    #undef  OSPRESERVE_INTERRUPT_MASK
    #define OSPRESERVE_INTERRUPT_MASK   TRUE 
    #undef  OSMSGQEMPTY_ATTR        
    #define OSMSGQEMPTY_ATTR            reentrant
#endif    
        
        
#if ( OSCALL_OSRETURNEVENT == OSFROM_ANYWHERE )
    #undef  OSPRESERVE_INTERRUPT_MASK
    #define OSPRESERVE_INTERRUPT_MASK   TRUE 
    #undef  OSRETURNBINSEM_ATTR            
    #define OSRETURNBINSEM_ATTR         reentrant
    #undef  OSRETURNEFLAG_ATTR            
    #define OSRETURNEFLAG_ATTR          reentrant
    #undef  OSRETURNMSG_ATTR            
    #define OSRETURNMSG_ATTR            reentrant
    #undef  OSRETURNMSGQ_ATTR            
    #define OSRETURNMSGQ_ATTR           reentrant
    #undef  OSRETURNSEM_ATTR            
    #define OSRETURNSEM_ATTR            reentrant
#endif 
   
               

#if ( OSCALL_OSSIGNALEVENT == OSFROM_ANYWHERE )
    #undef  OSPRESERVE_INTERRUPT_MASK
    #define OSPRESERVE_INTERRUPT_MASK   TRUE 
    #undef  OSSIGNALBINSEM_ATTR            
    #define OSSIGNALBINSEM_ATTR         reentrant
    #undef  OSSIGNALEFLAG_ATTR            
    #define OSSIGNALEFLAG_ATTR          reentrant
    #undef  OSSIGNALEVENT_ATTR            
    #define OSSIGNALEVENT_ATTR          reentrant
    #undef  OSSIGNALMSG_ATTR 
    #define OSSIGNALMSG_ATTR            reentrant
    #undef  OSSIGNALMSGQ_ATTR            
    #define OSSIGNALMSGQ_ATTR           reentrant
    #undef  OSSIGNALSEM_ATTR            
    #define OSSIGNALSEM_ATTR            reentrant
#endif    
                

#if ( OSCALL_OSSTARTTASK == OSFROM_ANYWHERE )
    #undef  OSPRESERVE_INTERRUPT_MASK
    #define OSPRESERVE_INTERRUPT_MASK   TRUE 
    #undef  OSSTARTTASK_ATTR            
    #define OSSTARTTASK_ATTR            reentrant
#endif    

#endif /* #if OSCOMPILER */

#endif /* __SALVOMCG_H */


/************************************************************
****                                                     ****
**                                                         **
Note that from this point forward, this file does NOT have
include guards. Neither do the related files salvolvl.h and
salvocfg.h.  That's because they are not blindly included
like a typical header file. Rather, they are "text insertion"
files, and may well appear more than once in each file that
includes them. An include guard would prevent the second and
later inclusions from having any effect. So, as long as there
are Salvo source files with that include them (indirectly or
directly) more than once, they cannot have include guards.

**                                                         **
****                                                     ****
************************************************************/

/************************************************************
****                                                     ****
**                                                         **
Keil C51 compiler. Reentrant functions must be NOAREG'd
so that they don't use / expect parameters in absolute
registers AR0-AR7.

**                                                         **
****                                                     ****
************************************************************/
#if ( OSCOMPILER == OSKEIL_C51 )

#if defined(__OSCREATEBINSEM_BINSEM_C) \
 || defined(__OSCREATEEFLAG_EFLAG_C)   \
 || defined(__OSCREATEEVENT_EVENT_C)   \
 || defined(__OSCREATEMSG_MSG_C)       \
 || defined(__OSCREATEMSGQ_MSGQ_C)     \
 || defined(__OSCREATESEM_SEM_C)  
    #if   OSCALL_OSCREATEEVENT == OSFROM_ANYWHERE
        #pragma NOAREGS
        #include <salvolvl.h>
    #endif
#endif


#if defined(__OSGETPRIOTASK_PRIO_C)
    #if   OSCALL_OSGETPRIOTASK == OSFROM_ANYWHERE
        #pragma NOAREGS
        #include <salvolvl.h>
    #endif
#endif


#if defined(__OSGETSTATETASK_TASK5_C)
    #if   OSCALL_OSGETSTATETASK == OSFROM_ANYWHERE
        #pragma NOAREGS
        #include <salvolvl.h>
    #endif
#endif


#if defined(__OSMSGQCOUNT_MSGQ4_C)
    #if   OSCALL_OSMSGQCOUNT == OSFROM_ANYWHERE
        #pragma NOAREGS
        #include <salvolvl.h>
    #endif
#endif


#if defined(__OSMSGQEMPTY_MSGQ3_C)
    #if   OSCALL_OSMSGQEMPTY == OSFROM_ANYWHERE
        #pragma NOAREGS
        #include <salvolvl.h>
    #endif
#endif


#if defined(__OSRETURNBINSEM_BINSEM2_C) \
 || defined(__OSRETURNEFLAG_EFLAG2_C)   \
 || defined(__OSRETURNMSG_MSG2_C)       \
 || defined(__OSRETURNMSGQ_MSGQ2_C)     \
 || defined(__OSRETURNSEM_SEM2_C)    
    #if   OSCALL_OSRETURNEVENT == OSFROM_ANYWHERE
        #pragma NOAREGS
        #include <salvolvl.h>
    #endif
#endif


#if defined(__OSSIGNALBINSEM_BINSEM_C) \
 || defined(__OSSIGNALEFLAG_EFLAG_C)   \
 || defined(__OSSIGNALEVENT_EVENT_C)   \
 || defined(__OSSIGNALMSG_MSG_C)       \
 || defined(__OSSIGNALMSGQ_MSGQ_C)     \
 || defined(__OSSIGNALSEM_SEM_C)    
    #if   OSCALL_OSSIGNALEVENT == OSFROM_ANYWHERE
        #pragma NOAREGS
        #include <salvolvl.h>
    #endif
#endif


#if defined(__OSSTARTTASK_TASK_C)
    #if   OSCALL_OSSTARTTASK == OSFROM_ANYWHERE
        #pragma NOAREGS
        #include <salvolvl.h>
    #endif
#endif


/************************************************************
****                                                     ****
**                                                         **
HI-TECH PICC, HI-TECH V8C compilers.

Note that the "blind" undef of OSBegin|EndCriticalSection() 
is compatible with the PIC12 PICmicros, since they don't 
have interrupts anyway.

**                                                         **
****                                                     ****
************************************************************/

#elif ( OSCOMPILER == OSHT_PICC ) || ( OSCOMPILER == OSHT_V8C )

/* OSCreateEvent() and OSCreateBinSem/EFlag/Msg/MsgQ/Sem() */
#if defined(__OSCREATEBINSEM_BINSEM_C) \
 || defined(__OSCREATEEFLAG_EFLAG_C)   \
 || defined(__OSCREATEEVENT_EVENT_C)   \
 || defined(__OSCREATEMSG_MSG_C)       \
 || defined(__OSCREATEMSGQ_MSGQ_C)     \
 || defined(__OSCREATESEM_SEM_C)  

    #undef OSBeginCriticalSection
    #undef OSEndCriticalSection
    #if   OSCALL_OSCREATEEVENT == OSFROM_BACKGROUND 
        #define OSBeginCriticalSection()      OSEnterCritical()
        #define OSEndCriticalSection()        OSLeaveCritical()
    #elif OSCALL_OSCREATEEVENT == OSFROM_FOREGROUND
        #define OSBeginCriticalSection() 
        #define OSEndCriticalSection()  
    #elif OSCALL_OSCREATEEVENT == OSFROM_ANYWHERE
        #include <salvolvl.h>
        #define OSBeginCriticalSection() 
        #define OSEndCriticalSection() 
    #endif
   
            
/* OSGetPrioTask() */
#elif defined(__OSGETPRIOTASK_PRIO_C)

    #undef OSBeginCriticalSection
    #undef OSEndCriticalSection
    #if   OSCALL_OSGETPRIOTASK == OSFROM_BACKGROUND
         #define OSBeginCriticalSection()  OSEnterCritical()
         #define OSEndCriticalSection()    OSLeaveCritical()
    #elif OSCALL_OSGETPRIOTASK == OSFROM_FOREGROUND
         #define OSBeginCriticalSection() 
         #define OSEndCriticalSection()  
    #elif OSCALL_OSGETPRIOTASK == OSFROM_ANYWHERE
         #include <salvolvl.h>
         #define OSBeginCriticalSection()
         #define OSEndCriticalSection()
    #endif


/* OSGetStateTask() */
#elif defined(__OSGETSTATETASK_TASK5_C)

    #undef OSBeginCriticalSection
    #undef OSEndCriticalSection
    #if   OSCALL_OSGETSTATETASK == OSFROM_BACKGROUND
         #define OSBeginCriticalSection()  OSEnterCritical()
         #define OSEndCriticalSection()    OSLeaveCritical()
    #elif OSCALL_OSGETSTATETASK == OSFROM_FOREGROUND
         #define OSBeginCriticalSection() 
         #define OSEndCriticalSection()  
    #elif OSCALL_OSGETSTATETASK == OSFROM_ANYWHERE
         #include <salvolvl.h>
         #define OSBeginCriticalSection()
         #define OSEndCriticalSection()  
     #endif


/* OSMsgQCount() */
#elif defined(__OSMSGQCOUNT_MSGQ4_C)

    #undef OSBeginCriticalSection
    #undef OSEndCriticalSection
    #if   OSCALL_OSMSGQCOUNT == OSFROM_BACKGROUND
         #define OSBeginCriticalSection()  OSEnterCritical()
         #define OSEndCriticalSection()    OSLeaveCritical()
    #elif OSCALL_OSMSGQCOUNT == OSFROM_FOREGROUND
         #define OSBeginCriticalSection() 
         #define OSEndCriticalSection()  
    #elif OSCALL_OSMSGQCOUNT == OSFROM_ANYWHERE
         #include <salvolvl.h>
         #define OSBeginCriticalSection()
         #define OSEndCriticalSection()
    #endif
    

/* OSMsgQEmpty() */
#elif defined(__OSMSGQEMPTY_MSGQ3_C)

    #undef OSBeginCriticalSection
    #undef OSEndCriticalSection
    #if   OSCALL_OSMSGQEMPTY == OSFROM_BACKGROUND
         #define OSBeginCriticalSection()  OSEnterCritical()
         #define OSEndCriticalSection()    OSLeaveCritical()
    #elif OSCALL_OSMSGQEMPTY == OSFROM_FOREGROUND
         #define OSBeginCriticalSection() 
         #define OSEndCriticalSection()  
    #elif OSCALL_OSMSGQEMPTY == OSFROM_ANYWHERE
         #include <salvolvl.h>
         #define OSBeginCriticalSection()
         #define OSEndCriticalSection()
    #endif
    

/* OSReturnBinSem/EFlag/Msg/MsgQ/Sem() */
#elif defined(__OSRETURNBINSEM_BINSEM2_C) \
 || defined(__OSRETURNEFLAG_EFLAG2_C)   \
 || defined(__OSRETURNMSG_MSG2_C)       \
 || defined(__OSRETURNMSGQ_MSGQ2_C)  \
 || defined(__OSRETURNSEM_SEM2_C)    

    #undef OSBeginCriticalSection
    #undef OSEndCriticalSection
    #if   OSCALL_OSRETURNEVENT == OSFROM_BACKGROUND 
        #define OSBeginCriticalSection()  OSEnterCritical()
        #define OSEndCriticalSection()    OSLeaveCritical()
    #elif OSCALL_OSRETURNEVENT == OSFROM_FOREGROUND
        #define OSBeginCriticalSection() 
        #define OSEndCriticalSection()  
    #elif OSCALL_OSRETURNEVENT == OSFROM_ANYWHERE
        #include <salvolvl.h>
        #define OSBeginCriticalSection()
        #define OSEndCriticalSection()
    #endif


/* OSSignalEvent() and OSSignalBinSem/EFlag/Msg/MsgQ/Sem() */
#elif defined(__OSSIGNALBINSEM_BINSEM_C) \
 || defined(__OSSIGNALEFLAG_EFLAG_C)   \
 || defined(__OSSIGNALEVENT_EVENT_C)   \
 || defined(__OSSIGNALMSG_MSG_C)       \
 || defined(__OSSIGNALMSGQ_MSGQ_C)     \
 || defined(__OSSIGNALSEM_SEM_C)    

    #undef OSBeginCriticalSection
    #undef OSEndCriticalSection
    #if   OSCALL_OSSIGNALEVENT == OSFROM_BACKGROUND 
        #define OSBeginCriticalSection()  OSEnterCritical()
        #define OSEndCriticalSection()    OSLeaveCritical()
    #elif OSCALL_OSSIGNALEVENT == OSFROM_FOREGROUND
        #define OSBeginCriticalSection() 
        #define OSEndCriticalSection()  
    #elif OSCALL_OSSIGNALEVENT == OSFROM_ANYWHERE
        #include <salvolvl.h>
        #define OSBeginCriticalSection()
        #define OSEndCriticalSection()
    #endif


/* OSStartTask() */
#elif defined(__OSSTARTTASK_TASK_C)  

    #undef OSBeginCriticalSection
    #undef OSEndCriticalSection
    #if   OSCALL_OSSTARTTASK == OSFROM_BACKGROUND 
        #define OSBeginCriticalSection()  OSEnterCritical()
        #define OSEndCriticalSection()    OSLeaveCritical()
    #elif OSCALL_OSSTARTTASK == OSFROM_FOREGROUND
        #define OSBeginCriticalSection() 
        #define OSEndCriticalSection()  
    #elif OSCALL_OSSTARTTASK == OSFROM_ANYWHERE
        #include <salvolvl.h>
        #define OSBeginCriticalSection()
        #define OSEndCriticalSection()
    #endif

#endif /* #if ... #elif ... */

/************************************************************
****                                                     ****
**                                                         **
ImageCraft compilers.

ICC requires the function name be listed on the same
line as the pragma, so we can't use the OSMONITOR_KEYWORD_PRE
version like IAR430 and AQ430.

**                                                         **
****                                                     ****
************************************************************/
#elif ( (OSCOMPILER == OSIMAGECRAFT) \
   &&   (OSMONITOR_KEYWORD_EXISTS == TRUE) )
   
   
/* common to ICC430 and ICCAVR */	
#if ( (OSTARGET == OSMSP430 ) || (OSTARGET == OSAVR) ) 

#if   defined(__OSCREATEBINSEM_BINSEM_C)
	#pragma monitor OSCreateBinSem
	
#elif defined(__OSWAITBINSEM_BINSEM_C)
	#pragma monitor OSWaitBinSem
	
#elif defined(__OSSIGNALBINSEM_BINSEM_C)
	#pragma monitor OSSignalBinSem
	
#elif defined(__OSCREATECYCTMR_CYCLIC_C)
	#pragma monitor OSCreateCycTmr
	
#elif defined(__OSSTARTCYCTMR_CYCLIC2_C)
	#pragma monitor OSStartCycTmr
	
#elif defined(__OSSTOPCYCTMR_CYCLIC3_C)
	#pragma monitor OSStopCycTmr
	
#elif defined(__OSDESTROYCYCTMR_CYCLIC4_C)
	#pragma monitor OSDestroyCycTmr
	
#elif defined(__OSSETCYCTMRPERIOD_CYCLIC5_C)
	#pragma monitor OSSetCycTmrPeriod
	
#elif defined(__OSRESETCYCTMR_CYCLIC6_C)
	#pragma monitor OSResetCycTmr
	
#elif defined(__OSCYCTMRRUNNING_CYCLIC7_C)
	#pragma monitor OSCycTmrRunning
	
#elif defined(__OSRETURNBINSEM_BINSEM2_C)
	#pragma monitor OSReturnBinSem
	
#elif defined(__OSDELAY_DELAY_C)
	#pragma monitor OSDelay
	
#elif defined(__OSSYNCTSTASK_DELAY2_C)
	#pragma monitor OSSyncTSTask
	
#elif defined(__OSGETTSTASK_DELAY3_C)
	#pragma monitor OSGetTSTask
	
#elif defined(__OSSETTSTASK_DELAY3_C)
	#pragma monitor OSSetTSTask
	
#elif defined(__OSDESTROY_DESTROY_C)
	#pragma monitor OSDestroy
	
#elif defined(__OSCREATEEFLAG_EFLAG_C)
	#pragma monitor OSCreateEFlag
	
#elif defined(__OSWAITEFLAG_EFLAG_C)
	#pragma monitor OSWaitEFlag
	
#elif defined(__OSSIGNALEFLAG_EFLAG_C)
	#pragma monitor OSSignalEFlag
	
#elif defined(__OSRETURNEFLAG_EFLAG2_C)
	#pragma monitor OSReturnEFlag
	
#elif defined(__OSCREATEEVENT_EVENT_C)
	#pragma monitor OSCreateEvent
	
#elif defined(__OSSIGNALEVENT_EVENT_C)
	#pragma monitor OSSignalEvent
	
#elif defined(__OSWAITEVENT_EVENT_C)
	#pragma monitor OSWaitEvent
	
#elif defined(__OSCREATETASK_INITTASK_C)
	#pragma monitor OSCreateTask
	
#elif defined(__OSCREATEMSG_MSG_C)
	#pragma monitor OSCreateMsg
	
#elif defined(__OSWAITMSG_MSG_C)
	#pragma monitor OSWaitMsg
	
#elif defined(__OSSIGNALMSG_MSG_C)
	#pragma monitor OSSignalMsg
	
#elif defined(__OSRETURNMSG_MSG2_C)
	#pragma monitor OSReturnMsg
	
#elif defined(__OSCREATEMSGQ_MSGQ_C)
	#pragma monitor OSCreateMsgQ
	
#elif defined(__OSWAITMSGQ_MSGQ_C)
	#pragma monitor OSWaitMsgQ
	
#elif defined(__OSSIGNALMSGQ_MSGQ_C)
	#pragma monitor OSSignalMsgQ
	
#elif defined(__OSRETURNMSGQ_MSGQ2_C)
	#pragma monitor OSReturnMsgQ
	
#elif defined(__OSMSGQCOUNT_MSGQ4_C)
	#pragma monitor OSMsgQCount
	
#elif defined(__OSMSGQEMPTY_MSGQ3_C)
	#pragma monitor OSMsgQEmpty
	
#elif defined(__OSSETPRIO_PRIO_C)
	#pragma monitor OSSetPrio
	
#elif defined(__OSSCHED_SCHED_C)
	#pragma monitor OSSched
	
#elif defined(__OSCREATESEM_SEM_C)
	#pragma monitor OSCreateSem
		
#elif defined(__OSWAITSEM_SEM_C)
	#pragma monitor OSWaitSem
		
#elif defined(__OSSIGNALSEM_SEM_C)
	#pragma monitor OSSignalSem
		
#elif defined(__OSRETURNSEM_SEM2_C)
	#pragma monitor OSReturnSem
		
#elif defined(__OSSTOP_STOP_C)
	#pragma monitor OSStop
		
#elif defined(__OSSTARTTASK_TASK_C)
	#pragma monitor OSStartTask
		
#elif defined(__OSSTOPTASK_TASK2_C)
	#pragma monitor OSStopTask
		
#elif defined(__OSDESTROYTASK_TASK3_C)
	#pragma monitor OSDestroyTask
		
#elif defined(__OSGETSTATETASK_TASK5_C)
	#pragma monitor OSGetStateTask
		
#elif defined(__OSSETPRIOTASK_TASK6_C)
	#pragma monitor OSSetPrioTask
		
#elif defined(__OSGETTICKS_TICK_C)
	#pragma monitor OSGetTicks
		
#elif defined(__OSSETTICKS_TICK_C)
	#pragma monitor OSSetTicks
		
#elif defined(__OSSAVERTNADDR_UTIL_C)
	#pragma monitor OSSaveRtnAddr
	
#endif /* common to ICC430 and ICCAVR */	
				
#endif  /* #if ... #elif ... */

#endif /* #if OSCOMPILER */


