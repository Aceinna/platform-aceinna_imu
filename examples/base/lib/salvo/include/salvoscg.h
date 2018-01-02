/************************************************************ 
Copyright (C) 1995-2008 Pumpkin, Inc. and its
Licensor(s). Freely distributable.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvoscg.h,v $
$Author: aek $
$Revision: 3.13 $
$Date: 2008-04-27 14:45:19-07 $

This file is used in conjunction with salvomcg.h. This
is the "single call graph" (i.e. scg) file, which is
used to undo the effects of salvomcg.h wherever it is
used.

For functions that can be called from the foreground and
the background simultaneously, some compilers need
special directives, etc. to handle the multiple call
graphs. This occurs when the parameter and auto variable
storage is not stack-based.  To use, the source code
should have the following form:

    #define __OSFNNAME_MODULENAME_C
    #include <salvomcg.h>
    Function body
    #include <salvoscg.h>
    #undef __OSFNNAME_MODULENAME_C
    
See comments in salvomcg.h on why this file doesn't have
include guards.


************************************************************/
#ifndef __SALVOSCG_H
#define __SALVOSCG_H

/************************************************************
****                                                     ****
**                                                         **
This "undoes" the internal interrupt control (critical
section protection) and other function-specific stuff 
for all those functions that are affected by OSCALL_OSXYZ, 
etc.

**                                                         **
****                                                     ****
************************************************************/
#if defined(__OSCREATEBINSEM_BINSEM_C)  \
 || defined(__OSCREATECYCTMR_CYCLIC_C)  \
 || defined(__OSCREATEEFLAG_EFLAG_C)    \
 || defined(__OSCREATEEVENT_EVENT_C)    \
 || defined(__OSCREATEMSG_MSG_C)        \
 || defined(__OSCREATEMSGQ_MSGQ_C)      \
 || defined(__OSCREATESEM_SEM_C)        \
 || defined(__OSGETPRIOTASK_PRIO_C)     \
 || defined(__OSGETSTATETASK_TASK5_C)   \
 || defined(__OSMSGQCOUNT_MSGQ4_C)      \
 || defined(__OSMSGQEMPTY_MSGQ3_C)      \
 || defined(__OSRETURNBINSEM_BINSEM2_C) \
 || defined(__OSRETURNEFLAG_EFLAG2_C)   \
 || defined(__OSRETURNMSG_MSG2_C)       \
 || defined(__OSRETURNMSGQ_MSGQ2_C)     \
 || defined(__OSRETURNSEM_SEM2_C)       \
 || defined(__OSSIGNALBINSEM_BINSEM_C)  \
 || defined(__OSSIGNALEFLAG_EFLAG_C)    \
 || defined(__OSSIGNALEVENT_EVENT_C)    \
 || defined(__OSSIGNALMSG_MSG_C)        \
 || defined(__OSSIGNALMSGQ_MSGQ_C)      \
 || defined(__OSSIGNALSEM_SEM_C)        \
 || defined(__OSSTARTTASK_TASK_C)      
     
    /* PICC, V8C: restore default values for      		*/
    /*  OSEnter/ExitCritical():                   		*/
	#if  ( OSCOMPILER == OSHT_PICC ) || ( OSCOMPILER == OSHT_V8C )
        #undef OSBeginCriticalSection
        #undef OSEndCriticalSection
        #define OSBeginCriticalSection()   OSEnterCritical() 
        #define OSEndCriticalSection()     OSLeaveCritical()
    #endif


    /* Keil C51: Undo #pragma NOAREGS            		*/
	#if  ( OSCOMPILER == OSKEIL_C51 )
        #pragma AREGS
    #endif

#endif 


#endif /* __SALVOSCG_H */
