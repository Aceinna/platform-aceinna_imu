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
places(TM). Copyright (C) 1995-2002 Pumpkin, Inc. and its
Licensor(s). All Rights Reserved.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Inc\\GCCARM\\salvoportgccarmcm3.h,v $
$Author: aek $
$Revision: 3.4 $
$Date: 2008-06-05 18:27:52-07 $

Porting file for GCC ARM and ARM Cortex-M3.

************************************************************/


#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>


//
// ========================================================
// Begin user macro definitions. If defined by the user in
//  the project's salvocfg.h configuration file, the user
//  definition will be used. If not, the default definition
//  (below) will be used.    
//              


//
// End user-configurable macro definitions.
// ========================================================
//

//
// ========================================================
// Begin compiler-specific macro definitions.
//


// Use OSDispatch() method with externally (i.e. user-)
//  defined interrupt control to ensure total target
//  independence of Salvo libraries.
// for ARM, we need to save the volatile registers per
// the ARM procedure call standard.  This macro defines
// the space reserved in the TCB for register storage
#define OSCTXSW_METHOD                 OSVIA_OSDISPATCH
#define OSCTXSW_SAVES_REGS             TRUE
#define OStypeSavedRegs OStypeInt32u
#define OSCTXSW_REGS_SAVED             8


// Because ARM Cortex-M3 is a 32-bit native machine, we
//  must ensure correct 8/16/32-bit typedefs.
#define OSINT_SIZE                     OSINT_IS_32BITS


// SP is 32 bits.
#define OSBYTES_OF_FRAMEP              4


// For 8-bit delay configurations, take advantage of
//  unused room in each tcb's status word to store the
//  delay field.
#define OSSTORE_8BIT_DELAYS_IN_STATUS  TRUE



// use weak hooks to avoid multiply defined symbol
//  errors between user functions and Salvo library
//  functions at link time.
#define OSUSE_WEAK_HOOKS               TRUE


// Should work fine.
#if !defined(OSUSE_MEMSET)
#define OSUSE_MEMSET                   TRUE
#endif


// printf() supports p (pointer) format.
//    NOTE: printf() support must be included in makefile
// and the user must provide the low-level character out
// routine. See avr-libc docs or <stdio.h> header file
// for more information.
#define OSUSE_PRINTF_P_FORMAT          TRUE
  
                
//
// End compiler-specific macro definitions.
// ========================================================
// 


/************************************************************
****                                                     ****
**                                                         **
Notes

1. OSDispatch() and OSCtxSw() are in CRTXM3\salvoportcrtxm3.S

**                                                         **
****                                                     ****
************************************************************/
