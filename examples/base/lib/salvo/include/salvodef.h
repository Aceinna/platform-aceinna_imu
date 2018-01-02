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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvodef.h,v $
$Author: aek $
$Revision: 3.21 $
$Date: 2008-05-13 09:21:00-07 $

Salvo defined constants.

************************************************************/
#ifndef __SALVODEF_H
#define __SALVODEF_H

/************************************************************
****                                                     ****
**                                                         **
#defines for use in salvocfg.h.

Salvocfg.h contains user-definable SALVO configurations.

**                                                         **
****                                                     ****
************************************************************/
#define FALSE                   0
#define TRUE                    (!FALSE)  

#define OSUNDEF                 0   /* generic const       */
#define OSNONE                  0   /*  ""                 */

#define OSA                     1   /* letter codes used   */
#define OSB                     2   /*  with libraries     */
#define OSC                     3   /*  etc.               */
#define OSD                     4   /*  ""                 */
#define OSE                     5   /*  ""                 */
#define OSF                     6   /*  ""                 */
#define OSG                     7   /*  ""                 */
#define OSH                     8   /*  ""                 */
#define OSI                     9   /*  ""                 */
#define OSJ                     10  /*  ""                 */
#define OSK                     11  /*  ""                 */
#define OSL                     12  /*  ""                 */
#define OSM                     13  /*  ""                 */
#define OSN                     14  /*  ""                 */
#define OSO                     15  /*  ""                 */
#define OSP                     16  /*  ""                 */
#define OSQ                     17  /*  ""                 */
#define OSR                     18  /*  ""                 */
#define OSS                     19  /*  ""                 */
#define OST                     20  /*  ""                 */
#define OSU                     21  /*  ""                 */
#define OSV                     22  /*  ""                 */
#define OSW                     23  /*  ""                 */
#define OSX                     24  /*  ""                 */
#define OSY                     25  /*  ""                 */
#define OSZ                     26  /*  ""                 */

                                    /* for OSLOGGING       */
#define OSLOG_NONE              0   /* no messages at all  */
#define OSLOG_ERRORS            1   /* error messages only */
#define OSLOG_WARNINGS          2   /* warn & err messages */
#define OSLOG_ALL               3   /* all messages        */
                                                                    
                                    /* for OSTARGET        */
#define OSPIC12                 1   /* Microchip PIC12     */
#define OSPIC16                 2   /* Microchip PIC16     */
#define OSPIC17                 3   /* Microchip PIC17     */
#define OSPIC18                 4   /* Microchip PIC18     */
#define OSIX86                  5   /* Intel x86           */
#define OSI8051                 6   /* Intel MCS-51        */
#define OSVAV8                  7   /* VAutomation V8-uRISC*/
#define OSM68HC11               8   /* Motorola 68HC11     */
#define OSMSP430                9   /* TI MSP430 series    */
#define OSM68HC12               10  /* Motorola 68HC12     */
#define OSM68HC08               11  /* Motorola 68HC08     */
#define OSEZ8                   12  /* Zilog Z8 Enhanced   */
#define OSTMS320C28X            13  /* TI TMS DSP '28x     */
#define OSAVR                   14  /* Atmel's AVR family  */
#define OSTMS320C24X            15  /* TI TMS DSP '24x     */
#define OSDSP56800              16  /* Moto DSP56800       */
#define OSSH2                   17  /* Hitachi SH2         */
#define OSM68HC08Z              18  /* 68HC08 all RAM in Zero Page */
#define OSM68HC05               19  /* Motorola 68HC05     */
#define OSM68HCS08              20  /* Freescale 68HCS08   */
#define OSARM7                  21  /* ARM7 core           */
#define OSPIC24F                22  /* Microchip PIC24F    */
#define OSPIC24H                23  /* Microchip PIC24H    */
#define OSDSPIC30F              24  /* Microchip dsPIC30F  */
#define OSDSPIC33F              25  /* Microchip dsPIC33H  */
#define OSARMCM3                26  /* ARM CM3 core        */
#define	OSPIC32                 27  /* Microchip PIC32     */
#define OSEPSON_S1C17           28  /* EPSON S1C17         */

                                    /* for OSCOMPILER      */
#define OSHT_PICC               1   /* HI-TECH PICC, PICC-18*/
#define OSMW_CW                 2   /* Metrowerks CodeWarrior*/
#define OSMIX_PC                3   /* Mix Power C         */
#define OSIAR_ICC               4   /* IAR C               */
#define OSMPLAB_C17             5   /* Microchip MPLAB C-17*/
#define OSMPLAB_C18             6   /* Microchip MPLAB C-18*/
#define OSHT_8051C              7   /* HI-TECH 8051C       */
#define OSHT_V8C                8   /* HI-TECH V8C         */
#define OSGCC                   9   /* GNU C               */
#define OSKEIL_C51              10  /* Keil C51            */
#define OSTASKING_CC51          11  /* TASKING CC51        */
#define OSAQ_430                12  /* Quadravox 430       */
#define OSIMAGECRAFT            13  /* ImageCraft C        */
#define OSZILOG_ZDSII           14  /* Zilog ZDS II        */
#define OSTI_DSP_CCS            15  /* TI Code Composer Studio */
#define OSRA_CROSSSTUDIO        16  /* Rowley Assoc. CS    */
#define OSMW_56800              17  /* MW for 56800        */
#define OSHPIT_CVAVR            18    /* HP InfoTech CodeVision AVR */
#define OSAVR_GCC               19  /* AVR GCC             */
#define OSGCC_AVR               OSAVR_GCC 
#define OSHT_430C               20  /* HI-TECH MSP430-C    */ 
#define OSHIT_SH2               21
#define OSCOSMIC                22  /* Cosmic 680x, etc.   */
#define OSKEIL_CARM             23    /* Keil CARM         */
#define OSARM_GCC               24  /* ARM GCC             */
#define OSGCC_ARM               OSARM_GCC
#define OSMPLAB_C30             25  /* Microchip MPLAB C30 */
#define OSGCC_CM3               26  /* GNU C for ARM       */
#define OSARM_RVDS              27  /* ARM RVDS toolchain  */
#define OSMPLAB_C32             28  /* Microchip MPLAB C32 */
#define OSGCC_S1C17             29  /* For EPSON S1C17     */
#define OSMSC_CPP               30  /* Microsoft C/C++     */

                                    /* abbreviations       */
#define OSFROM_ANYWHERE         OSA /* fns called from anywhere*/
#define OSFROM_BACKGROUND       OSB /* fns called normally */
#define OSFROM_TASK_ONLY        OSFROM_BACKGROUND
#define OSFROM_FOREGROUND       OSF /* fns called from ISRs only*/
#define OSFROM_ISR_ONLY         OSFROM_FOREGROUND


                                    /* OSCTXSW_METHOD      */
#define OSVIA_OSCTXSW           1   /* ctxSw via OSCtxSw() */
#define OSRTNADDR_IS_PARAM      2   /* OSrtnAddr is a param*/
#define OSRTNADDR_IS_VAR        3   /* OSrtnAddr is a var  */
#define OSVIA_OSDISPATCH        4   /* ctxSw via OSCtxSw() */
                                    /*  and use OSDispatch() */ 
#define OSVIA_OSDISPATCH_WLABEL 5   /* OSDispatch()+label  */                                                                    
#define OSVIA_OSDISPATCH_WPARAM 6   /* OSDispatch()+parameter */ 
                                    
                                    
                                    
#define OSINT_IS_16BITS            1
#define OSINT_IS_32BITS            2
                                   

/************************************************************ 
****                                                     ****
**                                                         **
Task Priorities:

16 separate task priorities are supported, from 0 =
highest priority to 15 = lowest. If the idle task is
enabled, we exclude its priority (lowest, 15) from
the range of available priorities.

By default, idle task has a tID of 1.  However, since v2.2.0
and earlier had the OSIDLE_TASK_NUM default to OSTASKS, the
user can override this setting if necessary.

**                                                         **
****                                                     ****
************************************************************/
#define OSHIGHEST_PRIO                  0
#define OSLOWEST_PRIO                   15

#define OSCT_ONE_SHOT                   0
#define OSCT_CONTINUOUS                 1
                                        
                                        
/************************************************************ 
****                                                     ****
**                                                         **
Task State Definitions:

 OSTCB_DESTROYED          uninitialized / dead. Uninitialized
                           only valid if compiler initializes all 
                           global vars to 0. Also represents
                           a completely cleared tcb (zero'd).
 OSTCB_TASK_STOPPED       task is stopped.
 OSTCB_TASK_DELAYED       task is delayed.
 OSTCB_TASK_SIGNALED      task has been signaled (used by
                            fast signaling)
 OSTCB_TASK_WAITING       task is waiting for event, with an
                            optional timeout
 OSTCB_TASK_ELIGIBLE      task is enqueued in the eligQ or 
                           running, normal.
 OSTCB_TASK_TIMED_OUT     task timed out waiting for an event.
 OSTCB_CYCLIC_TIMER       tcb is holding a cyclic timer.
 
A task is running when it is the current task and its state 
is OSTCB_TASK_ELIGIBLE.

**                                                         **
****                                                     ****
************************************************************/
#define OSTCB_DESTROYED                 0
#define OSTCB_TASK_STOPPED              1
#define OSTCB_TASK_DELAYED              2
#define OSTCB_TASK_SIGNALED             3
#define OSTCB_TASK_WAITING              4
#define OSTCB_TASK_ELIGIBLE             5
#define OSTCB_TASK_TIMED_OUT            6
#define OSTCB_CYCLIC_TIMER              7

#define OSTCB_TASK_RUNNING      8   /* this is never stored in */
                                    /*  the state, but is used */
                                    /*  by OSGetState[Task](). */


/************************************************************ 
****                                                     ****
**                                                         **
Generic No-timeout Definition:

**                                                         **
****                                                     ****
************************************************************/
#define OSNO_TIMEOUT                    0


/************************************************************ 
****                                                     ****
**                                                         **
Bit Masks:

Bit masks used to "extract" status or priority from the 
task control block.

By convention, masks preserve bits of interest and destroy 
those not of interest, via AND.

Three masks are not normally used -- rather, bitfields for
priority, state and running are defined in the task's
status byte, and are operated on as bitfields.

OSDONT_START_TASK is used with OSCreateTask() to leave the 
task in the stopped state after it has been created.

**                                                         **
****                                                     ****
************************************************************/
#define OSBITMASK_PRIO                  0x0F    
#define OSBITMASK_STATE                 0x70    
#define OSBITMASK_YIELDED               0x80

#define OSDONT_START                    0x80
#define OSDONT_START_TASK               0x80
#define OSDONT_START_CYCTMR             0x80
                                        

/************************************************************ 
****                                                     ****
**                                                         **
Error Codes:

**                                                         **
****                                                     ****
************************************************************/
#define OSNOERR                         0
#define OSERR                           1

#define OSERR_BAD_P                     2

#define OSERR_EVENT_NA                  3
#define OSERR_EVENT_FULL                4
#define OSERR_EVENT_CB_UNINIT           5
#define OSERR_EVENT_BAD_TYPE            6

#define OSERR_BAD_PRIO                  7
#define OSERR_BAD_STATE                 8

#define OSERR_BAD_CT                    9
#define OSERR_BAD_CT_MODE               10
#define OSERR_BAD_CT_DELAY              11
#define OSERR_CT_RUNNING                12
#define OSERR_CT_STOPPED                13


#define OSERR_NOYIELD                   1    /* return codes for */
#define OSERR_YIELD                     2    /*  OSWaitEvent().  */
#define OSERR_TIMEOUT                   4    /*  return value is */
#define OSERR_AVAILABLE                 8    /*  OR'd combo of   */
#define OSERR_SIGNALED                  16   /*  these values.   */
#define OSERR_SEVERE                    32   /* used internally  */


/************************************************************ 
****                                                     ****
**                                                         **
Defined Event Types:

Event types are always defined, but not necessarily
stored in the ecb.

**                                                         **
****                                                     ****
************************************************************/
#define OSEV_DESTROYED                  0   
#define OSEV_BINSEM                     1   
#define OSEV_EFLAG                      2    
#define OSEV_MSG                        3
#define OSEV_MSGQ                       4
#define OSEV_SEM                        5

#define OSANY_BITS                      1  /* bit positions */
#define OSALL_BITS                      2  /*  ""           */    
#define OSEXACT_BITS                    4  /*  ""           */ 

#define OSCLR_EFLAG                     1  /*  ""           */  
#define OSSET_EFLAG                     2  /*  ""           */  
                            
#endif /* __SALVODEF_H */
