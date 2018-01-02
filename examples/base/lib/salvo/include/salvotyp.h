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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvotyp.h,v $
$Author: aek $
$Revision: 3.12 $
$Date: 2008-04-27 14:45:18-07 $

Salvo native typedefs.

************************************************************/
#ifndef __SALVOTYP_H
#define __SALVOTYP_H

typedef unsigned char OStypeBoolean;   /* FALSE or TRUE       */
typedef signed char   OStypeInt8;      /* -128 to +127        */
typedef unsigned char OStypeInt8u;     /* 0 to +255           */
typedef const unsigned char OStypeConstInt8u;     /* 0 to +255           */


#if   OSINT_SIZE == OSINT_IS_16BITS
typedef signed int    OStypeInt16;     /* -32768 to +32767    */
typedef unsigned int  OStypeInt16u;    /* 0 to +65,535        */
typedef signed long   OStypeInt32;     /* -2.1B to +2.1B      */
typedef unsigned long OStypeInt32u;    /* 0 to +4,294,967,295 */
#elif OSINT_SIZE == OSINT_IS_32BITS
typedef signed short  OStypeInt16;     /* -32768 to +32767    */
typedef unsigned short OStypeInt16u;   /* 0 to +65,535        */
typedef signed int    OStypeInt32;     /* -2.1B to +2.1B      */
typedef unsigned int  OStypeInt32u;    /* 0 to +4,294,967,295 */
#else
#error salvotyp.h: Unknown native data type sizes (OSINT_SIZE).
#endif


typedef OStypeInt8u   OStypePrio;      /* 16 unique priorities */
#define OStypeCTMode  OStypePrio       /* synonymous           */
typedef OStypeInt8u   OStypeState;     /* 8 unique states      */
typedef OStypeInt8u   OStypeOption;    /* generic option type, */
                                       /*  e.g. for eFlgs      */

typedef OStypeInt8u   OStypeEType;     /* 256 event types      */
typedef OStypeInt8u   OStypeDepth;     /* 256 stack depth      */
typedef OStypeInt8u   OStypeID;        /* 256 IDs              */

typedef OStypeInt8u   OStypeErr;       /* 8-bit return (error) */
									   /*  codes               */

typedef struct ecb    OStypeEcb;       /* event control block  */
typedef struct tcb    OStypeTcb;       /* task control block   */
typedef struct mqcb   OStypeMqcb;      /* msg Q control block  */
typedef struct efcb   OStypeEfcb;      /* ev flag control block*/

typedef OSTYPE_TCBEXT0 OStypeTcbExt0;  /* user-definable tcb   */
typedef OSTYPE_TCBEXT1 OStypeTcbExt1;  /*  extension types     */
typedef OSTYPE_TCBEXT2 OStypeTcbExt2;  /*                      */
typedef OSTYPE_TCBEXT3 OStypeTcbExt3;  /*                      */
typedef OSTYPE_TCBEXT4 OStypeTcbExt4;  /*                      */
typedef OSTYPE_TCBEXT5 OStypeTcbExt5;  /*                      */


/* NOTE: OStypeBoolean, OStypeBinSem and OStypeSem                 */
/* (!OSBIG_SEMAPHORES) are all alike -- event processing expects   */
/*  and requires this.  See OSWaitXXX(). Also OStypeMsgQSize       */

typedef OStypeBoolean OStypeBinSem;    /* binSems are just 0 or 1  */

#if !OSBIG_SEMAPHORES                  /* small semaphores have a  */
typedef OStypeInt8u OStypeSem;         /*  maximum value of 255,   */
#define MAX_SEM 255                    /*  big ones 32,767         */
#else
typedef OStypeInt16u OStypeSem;
#define MAX_SEM 32767
#endif /* #if !OSBIG_SEMAPHORES  */

typedef OStypeInt8u OStypeMsgQSize;    /* 255 messages per queue   */
typedef OStypeMsgQSize * OStypeMsgQSizeP;

#if OSUSE_ARRAYS
#if OSARRAY_SIZE_IS_BYTE
typedef OStypeInt8u OStypePrioA;       /* size of priority arrays  */
#else                                  /*  depends on the number   */
typedef OStypeInt16u OStypePrioA;      /*  of tasks                */
#endif
#endif


/************************************************************
****                                                     ****
**                                                         **
Numeric Type Definitions:

**                                                         **
****                                                     ****
************************************************************/
#if   (OSBYTES_OF_COUNTS == 0) || (OSBYTES_OF_COUNTS == 1)
typedef OStypeInt8u  OStypeCount;
#elif OSBYTES_OF_COUNTS == 2
typedef OStypeInt16u OStypeCount;
#elif OSBYTES_OF_COUNTS == 4
typedef OStypeInt32u OStypeCount;
#else
#error salvotyp.h: Illegal value for OSBYTES_OF_COUNTS. Must be 0, 1, 2 or 4.
#endif

#if   (OSBYTES_OF_DELAYS == 0) || (OSBYTES_OF_DELAYS == 1)
typedef OStypeInt8u  OStypeDelay;
typedef OStypeInt8   OStypeInterval;
#elif OSBYTES_OF_DELAYS == 2
typedef OStypeInt16u OStypeDelay;
typedef OStypeInt16  OStypeInterval;
#elif OSBYTES_OF_DELAYS == 4
typedef OStypeInt32u OStypeDelay;
typedef OStypeInt32  OStypeInterval;
#else
#error salvotyp.h: Illegal value for OSBYTES_OF_DELAYS. Must be 0, 1, 2 or 4.
#endif

#define OStypeTS OStypeDelay /* synonymous */

#if   (OSBYTES_OF_EVENT_FLAGS == 0) || (OSBYTES_OF_EVENT_FLAGS == 1)
typedef OStypeInt8u  OStypeEFlag;
#elif OSBYTES_OF_EVENT_FLAGS == 2
typedef OStypeInt16u OStypeEFlag;
#elif OSBYTES_OF_EVENT_FLAGS == 4
typedef OStypeInt32u OStypeEFlag;
#else
#error salvotyp.h: Illegal value for OSBYTES_OF_EVENT_FLAGS. Must be 0, 1, 2 or 4.
#endif

#if   (OSBYTES_OF_FRAMEP == 0) || (OSBYTES_OF_FRAMEP == 1)
typedef OStypeInt8u  OStypeFrameP;
#elif OSBYTES_OF_FRAMEP == 2
typedef OStypeInt16u OStypeFrameP;
#elif OSBYTES_OF_FRAMEP == 4
typedef OStypeInt32u OStypeFrameP;
#else
#error salvotyp.h: Illegal value for OSBYTES_OF_FRAMEP. Must be 0, 1, 2 or 4.
#endif

#if   (OSBYTES_OF_TICKS == 0) || (OSBYTES_OF_TICKS == 1)
typedef OStypeInt8u  OStypeTick;
#elif OSBYTES_OF_TICKS == 2
typedef OStypeInt16u OStypeTick;
#elif OSBYTES_OF_TICKS == 4
typedef OStypeInt32u OStypeTick;
#else
#error salvotyp.h: Illegal value for OSBYTES_OF_TICKS. Must be 0, 1, 2 or 4.
#endif

#if   (OSBYTES_OF_SRGIE == 0) || (OSBYTES_OF_SRGIE == 1)
typedef OStypeInt8u  OStypeSRGIE;
#elif OSBYTES_OF_SRGIE == 2
typedef OStypeInt16u OStypeSRGIE;
#elif OSBYTES_OF_SRGIE == 4
typedef OStypeInt32u OStypeSRGIE;
#else
#error salvotyp.h: Illegal value for OSBYTES_OF_SRGIE. Must be 0, 1, 2 or 4.
#endif

#if   OSTIMER_PRESCALAR > 65535u
typedef OStypeInt32u OStypePS;
#elif OSTIMER_PRESCALAR > 255u
typedef OStypeInt16u OStypePS;
#elif OSTIMER_PRESCALAR >= 0
typedef OStypeInt8u  OStypePS;
#endif


/************************************************************
****                                                     ****
**                                                         **
Bitfield typedef.

Bitfields in ANSI C are normally of type signed int or
unsigned int. MISRA-C requires this.

Some compiler support bitfield packing, and we take
advantage of that on occasion.

Note that this has severe ramifications on certain Salvo
context switchers ...

**                                                         **
****                                                     ****
************************************************************/
#if OSUSE_CHAR_SIZED_BITFIELDS
typedef OStypeInt8u   OStypeBitField;       
#else                                      
typedef unsigned int  OStypeBitField;      
#endif


/************************************************************
****                                                     ****
**                                                         **
Port-specific typedefs.

These mainly have to do with differences among 8-bit
compilers w/regard to bitfields and pointer types.

**                                                         **
****                                                     ****
************************************************************/
typedef struct {                            /* status consists of:   */
    OStypeBitField prio:4;                  /*  4 bits of priority   */
    OStypeBitField state:3;                 /*  3 bits of state      */
    OStypeBitField yielded:1;               /*  1 yielded bit        */
    #if ( (OSCTXSW_METHOD == OSVIA_OSDISPATCH) \
      ||  (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WLABEL) \
      ||  (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WPARAM)  )
    #if !OSSTORE_8BIT_DELAYS_IN_STATUS
    OStypeBitField frameSize:8;             /*  8 bits of stack frame*/
                                            /*   size (only some     */
                                            /*   distributions)      */
    #else
    OStypeBitField delayPlaceHolder:8;
	OStypeBitField frameSize:16;            /* 16 bits of stack frame*/
                                            /*  size (only some 32-  */
                                            /*  bit distributions)   */
    #endif
    #endif
    } OStypeStatus;

typedef OStypeBitField OStypeStatusValue;   /* this is used where    */
                                            /*  a compiler can't     */
                                            /*  properly handle bit- */
                                            /*  field operations.    */
                                            /* Highly non-portable,  */
                                            /*  be careful!          */

typedef OStypeInt8u  OStypeLostTick;		/* no point in tracking  */
											/*  more than 8 bits of  */
											/*  lost ticks			 */

typedef OSMESSAGE_TYPE OStypeMsg;           /* usually void          */

#if (OSUSE_VOID_FN_POINTERS)
typedef void (* OStypeTFP) (void);
#endif


/************************************************************
****                                                     ****
**                                                         **
Unqualified and qualified typedefs.

So far all the supported compilers follow ANSI by accepting
memory type qualifiers after the reserved types and before
the pointer *.

**                                                         **
****                                                     ****
************************************************************/
#if 1 // 1: use typedefs, 0: use #defines

typedef unsigned char    OSLOC_ECB *     OStypeCharEcbP;
typedef unsigned char    OSLOC_TCB *     OStypeCharTcbP;
typedef OStypeEcb        OSLOC_ECB *     OStypeEcbP;
typedef OStypeEfcb       OSLOC_EFCB *    OStypeEfcbP;
typedef OStypeMqcb       OSLOC_MQCB *    OStypeMqcbP;
typedef OStypeMsg *                      OStypeMsgP;
typedef OStypeMsg * *                    OStypeMsgPP;
typedef OStypeMsg *      OSLOC_MSGQ *    OStypeMsgQPP;
typedef OStypeTcb        OSLOC_TCB *     OStypeTcbP;
typedef OStypeTcb        OSLOC_TCB *     OSLOC_ECB * OStypeTcbPP;

typedef OStypeCount      OSLOC_COUNT     OSgltypeCount;
typedef OStypeDepth      OSLOC_DEPTH     OSgltypeDepth;
typedef OStypeEcb        OSLOC_ECB       OSgltypeEcb;
typedef OStypeEfcb       OSLOC_EFCB      OSgltypeEfcb;
typedef OStypeErr        OSLOC_ERR       OSgltypeErr;
typedef OStypeLostTick   OSLOC_LOST_TICK OSgltypeLostTick;
typedef OStypeInt8u      OSLOC_LOGMSG    OSgltypeLogMsg;
typedef OStypeMqcb       OSLOC_MQCB      OSgltypeMqcb;
typedef OStypePS         OSLOC_PS        OSgltypePS;
typedef OStypeTcb        OSLOC_TCB       OSgltypeTcb;
typedef OStypeTick       OSLOC_TICK      OSgltypeTick;

typedef OStypeTcbP       OSLOC_CTCB      OSgltypeCTcbP;
typedef OStypeEcbP       OSLOC_ECB       OSgltypeEcbP;
typedef OStypeMsgP       OSLOC_MSGQ      OSgltypeMsgQP;
typedef OStypeTcbP       OSLOC_SIGQ      OSgltypeSigQP;
typedef OStypeTcbP       OSLOC_ECB       OSgltypeTcbP;

typedef OStypeFrameP                     OSgltypeFrameP;
typedef OStypeSRGIE                      OSgltypeSRGIE;

#else

#define OStypeCharEcbP   unsigned char   OSLOC_ECB *
#define OStypeCharTcbP   unsigned char   OSLOC_TCB *
#define OStypeEcbP       OStypeEcb       OSLOC_ECB *
#define OStypeEfcbP      OStypeEfcb      OSLOC_EFCB *
#define OStypeMqcbP      OStypeMqcb      OSLOC_MQCB *
#define OStypeMsgP       OStypeMsg     *
#define OStypeMsgPP      OStypeMsg     * *
#define OStypeMsgQPP     OStypeMsg     * OSLOC_MSGQ *
#define OStypePrioAP     OStypePrioA     OSLOC_ECB *
#define OStypeTcbP       OStypeTcb       OSLOC_TCB *
#define OStypeTcbPP      OStypeTcb       OSLOC_TCB *  OSLOC_ECB *
#define OSgltypeCount    OStypeCount     OSLOC_COUNT
#define OSgltypeDepth    OStypeDepth     OSLOC_DEPTH
#define OSgltypeEcb      OStypeEcb       OSLOC_ECB
#define OSgltypeEfcb     OStypeEfcb      OSLOC_EFCB
#define OSgltypeErr      OStypeErr       OSLOC_ERR
#define OSgltypeLostTick OStypeLostTick  OSLOC_LOST_TICK
#define OSgltypeLogMsg   char            OSLOC_LOGMSG
#define OSgltypeMqcb     OStypeMqcb      OSLOC_MQCB
#define OSgltypePrioA    OStypePrioA     OSLOC_ECB
#define OSgltypePS       OStypePS        OSLOC_PS
#define OSgltypeTcb      OStypeTcb       OSLOC_TCB
#define OSgltypeTick     OStypeTick      OSLOC_TICK

#define OSgltypeCTcbP    OStypeTcbP      OSLOC_CTCB
#define OSgltypeEcbP     OStypeEcbP      OSLOC_ECB
#define OSgltypeMsgQP    OStypeMsgP      OSLOC_MSGQ
#define OSgltypeSigQP    OStypeTcbP      OSLOC_SIGQ
#define OSgltypeTcbP     OStypeTcbP      OSLOC_ECB

#define OSgltypeFrameP   OStypeFrameP
#define OSgltypeSRGIE    OStypeSRGIE

#endif


#endif /* __SALVOTYP_H */
