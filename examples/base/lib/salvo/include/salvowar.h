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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvowar.h,v $
$Author: aek $
$Revision: 3.6 $
$Date: 2008-04-27 14:45:18-07 $

Salvo port-specific workarounds.

************************************************************/
#ifndef __SALVOWAR_H
#define __SALVOWAR_H

#if !defined(OSUSE_VOID_FN_POINTERS)
#define OSUSE_VOID_FN_POINTERS			TRUE
#endif

#if !defined(OSUSE_CUSTOM_TFP_FIELD)
#define OSUSE_CUSTOM_TFP_FIELD			FALSE
#endif

#if !defined(OSENABLE_DEBUGGING_PRINTFS)
#define OSENABLE_DEBUGGING_PRINTFS      TRUE
#endif




/************************************************************
****                                                     ****
**                                                         **
Packaged defines for dealing with compiler incompatibilities.
Each symbol represents a particular bug found, and is used
to invoke a workaround. These are provided because users
may not have a current revision of the specified compiler.

All workarounds default to FALSE to ensure that the symbols
are always defined.

**                                                         **
****                                                     ****
************************************************************/
/* all workarounds are initially defined as FALSE to avoid */
/*  "unknown identifier" problems with unrelated compilers.*/
#define OSWORKAROUND_HT_PICC_1          FALSE
#define OSWORKAROUND_HT_V8C_2           FALSE


/* now begin compiler-specific workaround section. */
#if ( OSCOMPILER == OSHT_PICC ) || ( OSCOMPILER == OSHT_V8C )

#if !defined(_HTC_VER_MAJOR_)
#define _HTC_VER_MAJOR_                 0
#endif
#if !defined(_HTC_VER_MINOR_)
#define _HTC_VER_MINOR_                 0
#endif
#if !defined(_HTC_VER_PATCH_)
#define _HTC_VER_PATCH_                 0
#endif
#define OSHTC_VER                         (_HTC_VER_MAJOR_*1000) \
                                        + (_HTC_VER_MINOR_*  10) \
                                        + (_HTC_VER_PATCH_*   1)

#endif


/* version tracking for HI-TECH PICC compilers began with   */
/*  v7.87.                                                  */
#if ( OSCOMPILER == OSHT_PICC )
 
/* PICC workaround #1 -- on PIC12, functions that return an */
/*  8-bit value and have parameters MUST return a value     */
/*  immediately after a comparison with one of the param-   */
/*  eters. O/wise undefined symbol:btemp error occurs. This */
/*  bug was present in v7.87 and earlier and has not been   */
/*  fixed, TTBOMK.                                          */
#if ( OSTARGET == OSPIC12 )
#undef  OSWORKAROUND_HT_PICC_1
#define OSWORKAROUND_HT_PICC_1          TRUE
#endif

#endif /* HI-TECH PICC compilers */


/* version tracking for HI-TECH V8C compilers began with    */
/*  v7.85PL1.                                               */
#if ( OSCOMPILER == OSHT_V8C )

 
/* V8C workaround #1. "Can't generate code for this         */
/*  expression" when a pointer parameter is used to point   */
/*  to a field in an rvalue. Rendered obsolete 12/6/02      */
/*  because of the way we were using #define and #undef to  */
/*  implement this solution.                                */
#if OSHTC_VER <= 7850
#error salvo.h You must upgrade to V8C v7.85PL1 or later ...
#endif

/* V8C workaround #2. Doing:                                 */
/*      tcbP->status.bits.prio = prio                       */
/* results in a huge number of incorrect instructions.        */
/* Not yet fixed in V8C.                                    */
#undef  OSWORKAROUND_HT_V8C_2
#define OSWORKAROUND_HT_V8C_2            TRUE

#endif /* HI-TECH V8C compiler */


#endif /* __SALVOWAR_H */
