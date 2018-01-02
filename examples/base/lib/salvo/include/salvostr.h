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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvostr.h,v $
$Author: aek $
$Revision: 3.13 $
$Date: 2008-05-13 11:07:02-07 $

Salvo global structures.

************************************************************/
#ifndef __SALVOSTR_H
#define __SALVOSTR_H

/************************************************************ 
****                                                     ****
**                                                         **
Task Control Block (tcb) structure:

 tcbP:          start / resume address of task
 ctxSwVect:     additional words required in some x86 
                 implementations to enable the scheduler to 
                 dispatch the task
 state:         b7:    0: task has not yielded 
                       1: task has already yielded
                b6-b4: task state (DESTROYED, ELIGIBLE, etc.)
                b3-b0: task priority
 nextTcbP:      ptr to next tcb in eligible or event queue
 prevTcbP:            "  previous         "
 ecbP:          ptr to ecb of event queue this task is in
 runStatus:     a copy of the current task's state when it was
                  made ready to tun. Is used only while task
                  is running (i.e. is the current task). 
 nextDlyTcbP:   ptr to next tcb in delay queue
 prevDlyTcbP:     "    previous
 delay:         delay in ticks for waiting task
 
Note: for some implementations (e.g. x86), order is 
important!
 
**                                                         **
****                                                     ****
************************************************************/
struct tcb {
	/* status bits/fields and task's function pointer */
    #if OSCOMPILER == OSMIX_PC
    OStypeTFP tFP;
    long int ctxSwVect;    
    union {
        OStypeStatus      bits;
        OStypeStatusValue value;
    } status;    
        
    #else /* #if OSCOMPILER */
    union {
        OStypeStatus      bits;
        OStypeStatusValue value;
    } status;  
    union 
    {
    	#if OSUSE_CUSTOM_TFP_FIELD
    	OStypeRawTFP rawTFP;
    	#endif
        OStypeTFP tFP;	
    } u3;
    #endif /* #if OSCOMPILER */    
    
    #if OSCTXSW_SAVES_REGS  
    OStypeSavedRegs savedRegs[OSCTXSW_REGS_SAVED]; 
    #endif
   
    /* pointer to next tcb in priority queue */ 
    #if OSUSE_ARRAYS
    #if OSOPTIMIZE_FOR_SPEED
    OStypePrioA prioABits;
    #endif /* #if OSOPTIMIZE_FOR_SPEED */
    #else
    union
    {
		OStypeTcbP nextTcbP;
		OStypeStatus runStatus;
	} u2;  
    #endif /* #if OSUSE_ARRAYS */

    
    /* pointer to ecb, plus runStatus and cycTmr stuff, */
    /*  and next tcb in delay queue						*/        
    #if OSUSE_ARRAYS
    #if OSENABLE_DELAYS
    #if OSENABLE_TIMEOUTS
    union {                            
        OStypeEcbP ecbP;        
        #if OSENABLE_CYCLIC_TIMERS
        OStypeDelay period;
        #endif      
    } u1; 
    #endif /* #if OSENABLE_TIMEOUTS */    
    
    OStypeTcbP nextDlyTcbP;            
    #endif /* #if OSENABLE_DELAYS */
    #else /* #if OSUSE_ARRAYS */
    #if OSENABLE_TIMEOUTS
    union {                            
        OStypeEcbP ecbP;        
        OStypeStatus runStatus;        
        #if OSENABLE_CYCLIC_TIMERS
        OStypeDelay period;
        #endif      
    } u1; 
    
    OStypeTcbP nextDlyTcbP;            
    #endif /* #if OSENABLE_TIMEOUTS */    
    #endif /* #if OSUSE_ARRAYS */


    /* interval services require system ticks. */
    #if OSENABLE_DELAYS && !OSENABLE_TICKS
    union {
        OStypeDelay delay;                    
    } dly; 
    #elif OSENABLE_DELAYS && OSENABLE_TICKS
    union {
        OStypeDelay delay;    
        OStypeTS    timestamp;                
    } dly;
    #endif
    
    
    #if OSENABLE_FAST_SIGNALING
    #if OSENABLE_MESSAGES || OSENABLE_MESSAGE_QUEUES
    OStypeMsgP msgP;
    #endif
    #endif
    
    
    /* tcb extensions */
    #if OSENABLE_TCBEXT0
    OStypeTcbExt0 tcbExt0;
    #endif
    #if OSENABLE_TCBEXT1
    OStypeTcbExt1 tcbExt1;
    #endif
    #if OSENABLE_TCBEXT2
    OStypeTcbExt2 tcbExt2;
    #endif
    #if OSENABLE_TCBEXT3
    OStypeTcbExt3 tcbExt3;
    #endif
    #if OSENABLE_TCBEXT4
    OStypeTcbExt4 tcbExt4;
    #endif
    #if OSENABLE_TCBEXT5
    OStypeTcbExt5 tcbExt5;
    #endif

};


/************************************************************ 
****                                                     ****
**                                                         **
Message Queue Control Block (mqcb) structure:

 count:     number of messages in queue
 inPP:        message insertion point pointer
 outPP:     message removal point pointer
 beginPP:   start of message queue 
 endPP:        end of message queue
 
The message queue is a simple ring buffer of pointers to
elements of type void. 

**                                                         **
****                                                     ****
************************************************************/
struct mqcb {
    OStypeMsgQSize  count;
    OStypeMsgQPP    inPP;
    OStypeMsgQPP    outPP;
    OStypeMsgQPP    beginPP;
    OStypeMsgQPP    endPP;
};


/************************************************************ 
****                                                     ****
**                                                         **
Event Flag Control Block (efcb) structure:

 flag:    flag
 
The key is a bitfield which is compared against the mask
with an operator. Some operators combine the key with mask2.

**                                                         **
****                                                     ****
************************************************************/
struct efcb {
    OStypeEFlag eFlag;
};


/************************************************************ 
****                                                     ****
**                                                         **
Event structure:

 binSem:  binary semaphore (0, 1)
 msgP:    message pointer (can point to anything)
 mqcbP:   message queue control block pointer
 sem:     counting semaphore (0, ..., MAX_SEM)
              

Event Control Block (ecb) structure:

 tcbP:    tcb ptr to first task in this event queue
 val:     union field for the "meat" of the event
 type:    type of event (semaphore, etc.) (optional)
 
**                                                         **
****                                                     ****
************************************************************/
#if OSENABLE_EVENTS
typedef union {
	#if OSENABLE_BINARY_SEMAPHORES
    OStypeBinSem binSem;
    #endif
    
    #if OSENABLE_EVENT_FLAGS
    OStypeEfcbP  efcbP;
    #endif
    
    #if OSENABLE_MESSAGES
    OStypeMsgP   msgP;
    #endif
    
    #if OSENABLE_MESSAGE_QUEUES
    OStypeMqcbP  mqcbP;
    #endif
    
    #if OSENABLE_SEMAPHORES
    OStypeSem    sem;
    #endif
} OStypeEvent;

 
struct ecb {
    #if OSUSE_ARRAYS
    OStypePrioA tcbP;
    #else
    OStypeTcbP tcbP;
    #endif
    
    OStypeEvent event;
     
    #if OSUSE_EVENT_TYPES
    OStypeInt8u type;
    #endif
};
#endif


#endif /* __SALVOSTR_H */
