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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvopsh.h,v $
$Author: aek $
$Revision: 3.27 $
$Date: 2008-06-20 11:37:39-07 $

Salvo port-specific header file includes.

************************************************************/
#ifndef __SALVOPSH_H
#define __SALVOPSH_H

/************************************************************ 
****                                                     ****
**                                                         **
#include <portXyz.h>:

Include the compiler- and target processor-specific items
required for your system.

**                                                         **
****                                                     ****
************************************************************/
/* undefined compiler -- was not set by user */
#if !defined(OSCOMPILER)
#define OSCOMPILER                      OSUNDEF
#endif



/* Zilog ZDSII compiler for Z8 Encore. Put this here because */
/*  of bug in v4.2 in #if ... #elif ... #else ... #endif     */
/*  statements ... but it still screws up on the #else part. */
#if (OSCOMPILER == OSZILOG_ZDSII)

#include <osez8.h>



/* Cosmic compilers */
#elif (OSCOMPILER == OSCOSMIC)
#if (OSTARGET == OSM68HC08) || (OSTARGET == OSM68HC08Z) \
  || (OSTARGET == OSM68HCS08)
#include <oscsmc08.h>
#else
#error salvopsh.h: Undefined or Illegal Cosmic C Target
#endif


/* Your Compiler - specific items */
#elif   (OSCOMPILER == OSUNDEF)
  
#include <osuser.h>


/* Archelon/Quadravox 430 */
#elif (OSCOMPILER == OSAQ_430)

#include <osaq430.h>


/* GNU C compiler */
#elif (OSCOMPILER == OSGCC)

#include <osgcc.h>


/* ARM GCC C compiler (currently via Rowley CrossStudio for ARM only */
#elif (OSCOMPILER == OSGCC_ARM)

#include <GCCARM\salvoportgccarm.h>


/* HI-TECH 8051C compiler  */
#elif (OSCOMPILER == OSHT_8051C)

#include <osht51.h>


/* HI-TECH MSP430-C compiler  */
#elif (OSCOMPILER == OSHT_430C)

#include <osht430.h>


/* HI-TECH PICC and PICC-18 */
#elif (OSCOMPILER == OSHT_PICC)

#include <oshtpicc.h>


/* HI-TECH V8C compiler */
#elif (OSCOMPILER == OSHT_V8C)

#include <oshtv8.h>


/* IAR PICMicro C Compiler */
#elif (OSCOMPILER == OSIAR_ICC)

#if   (OSTARGET == OSARM7)
#include <IARARM\salvoportiararm.h>
#elif (OSTARGET == OSAVR)
#include <IARAVR\salvoportiaravr.h>
#elif (OSTARGET == OSMSP430)
#include <IAR430\salvoportiar430.h>
#elif (OSTARGET == OSPIC18)
#include <osiar18.h>
#else
#error salvo.h: salvopsh.h: Unknown target for IAR compiler.
#endif




/* ImageCraft compilers */
#elif (OSCOMPILER == OSIMAGECRAFT) 

#if   (OSTARGET == OSAVR)
#include <ICCAVR\salvoporticcavr.h>
#elif (OSTARGET == OSM68HC11)
#include <osicc11.h>
#elif (OSTARGET == OSMSP430)
#include <osicc430.h>
#elif (OSTARGET == OSARM7)
#include <ICCARM\salvoporticcarm.h>
#else
#error salvo.h: salvopsh.h: Unknown target for ImageCraft compiler.
#endif


/* AVR GCC compiler */
#elif ( (OSCOMPILER == OSAVR_GCC) || (OSCOMPILER == OSGCC_AVR) )

#include <GCCAVR\salvoportgccavr.h>

 
/* GNU17 compiler */
#elif ( (OSCOMPILER == OSGCC_S1C17) && (OSTARGET == OSEPSON_S1C17) )

#include <EPS1C17\salvoporteps1c17.h>

 
/* Keil C51 compiler */
#elif (OSCOMPILER == OSKEIL_C51)

#include <KC51\salvoportkc51.h>

 
/* Keil C51 compiler */
#elif (OSCOMPILER == OSKEIL_CARM)

#include <KCARM\salvoportkcarm.h>

 
/* Metrowerks CodeWarrior for i86 */
#elif (OSCOMPILER == OSMW_CW)

#include <salvoportmwcw.h>


/* Microsoft C/C++ compiler */
#elif (OSCOMPILER == OSMSC_CPP)

#if   (OSTARGET == OSIX86)
#include <MSCX86\salvoportmscx86.h>
#else
#error salvo.h: salvopsh.h: Unknown target for Microsoft C/C++ compiler.
#endif

  
    
/* Metrowerks CodeWarrior for Motorola DSP56800 */
#elif ( (OSCOMPILER == OSMW_56800) && (OSTARGET == OSDSP56800) )

#include <osmw56800.h>
  
    
/* Microchip MPLAB C-18 Compiler */
#elif (OSCOMPILER == OSMPLAB_C18)

#include <osmcc18.h>


/* Microchip MPLAB C30 Compiler */
#elif (OSCOMPILER == OSMPLAB_C30)

#include <MCC30\salvoportmcc30.h>


/* Microchip MPLAB C32 Compiler */
#elif (OSCOMPILER == OSMPLAB_C32)

#include <MCC32\salvoportmcc32.h>


/* Mix Power C */
#elif (OSCOMPILER == OSMIX_PC)

#include <osmixc.h>


/* TASKING CC51 compiler */
#elif (OSCOMPILER == OSTASKING_CC51)

#include <oscc51.h>

/* CodeSourcery GCC compiler for ARM v6t2 */
#elif ((OSCOMPILER == OSGCC_CM3) && (OSTARGET == OSARMCM3))

#include <GCCARM/salvoportgccarmcm3.h>

/* ARM compiler for CM3 */
#elif ((OSCOMPILER == OSARM_RVDS) \
  && ((OSTARGET == OSARM7) ||(OSTARGET == OSARMCM3)))

#include <ARMRV/salvoportarmrv.h>


/* TI Code Composer Studio compiler for TMS320C24x|28x */
#elif (OSCOMPILER == OSTI_DSP_CCS)

#if ( (OSTARGET == OSTMS320C24X) || (OSTARGET == OSTMS320C28X) )
#include <ostic2000.h>
#else
#error salvo.h: salvopsh.h: Unknown target for TI DSP CCS.
#endif


/* Rowley Associates CrossStudio */
#elif (OSCOMPILER == OSRA_CROSSSTUDIO)

#if (OSTARGET == OSMSP430)
#include <RA430\salvoportra430.h>
#else
#error salvo.h: salvopsh.h: Unknown target for CrossStudio compiler.
#endif


/* Hitahi Embedded Workbench for SH2 */
#elif (OSCOMPILER == OSHIT_SH2)

#include <oshitsh2.h>


/* fall-through on undefined or unknown OSCOMPILER */
#else
#error salvo.h: salvopsh.h: No known porting header file for this OSCOMPILER.


#endif /* #if OSCOMPILER */


/************************************************************ 
****                                                     ****
**                                                         **
Port-specific symbol defaults.

These are a safety net in case the symbol is not set in
portxxxx.h and the default is not desired.

**                                                         **
****                                                     ****
************************************************************/
#if !defined(OSBYTES_OF_FRAMEP)
#define OSBYTES_OF_FRAMEP               2 
#endif 

#if !defined(OSBYTES_OF_SRGIE)
#define OSBYTES_OF_SRGIE                2 
#endif 

#if !defined(OSCTXSW_METHOD)
#define OSCTXSW_METHOD                  OSVIA_OSCTXSW
#endif

#if !defined(OSINT_SIZE)
#define	OSINT_SIZE						OSINT_IS_16BITS
#endif

#if !defined(OSLOC_DEFAULT)
#define OSLOC_DEFAULT
#endif

#if !defined(OSMESSAGE_TYPE)
#define OSMESSAGE_TYPE                  void
#endif

#if !defined(OSMONITOR_KEYWORD_PRE) && !defined(OSMONITOR_KEYWORD_POST)
#define OSMONITOR_KEYWORD_EXISTS        FALSE
#define    OSMONITOR_KEYWORD_PRE
#define    OSMONITOR_KEYWORD_PRE_PROTO
#define    OSMONITOR_KEYWORD_POST
#define    OSMONITOR_KEYWORD_POST_PROTO
#else
#define OSMONITOR_KEYWORD_EXISTS        TRUE
#if   !defined(OSMONITOR_KEYWORD_PRE)
#define OSMONITOR_KEYWORD_PRE
#define OSMONITOR_KEYWORD_PRE_PROTO
#elif !defined(OSMONITOR_KEYWORD_POST)
#define OSMONITOR_KEYWORD_POST
#define OSMONITOR_KEYWORD_POST_PROTO
#endif
#endif

#if !defined(OSProtect)
#define OSProtect()
#endif

#if !defined(OSRtnToOSSched)
#define OSRtnToOSSched()                return
#endif
                                        
#if !defined(OSRTNADDR_OFFSET)
#define OSRTNADDR_OFFSET                0
#endif
                                        
#if !defined(OSSCHED_RETURN_LABEL)
#define OSSCHED_RETURN_LABEL()
#endif

#if !defined(OSSTORE_8BIT_DELAYS_IN_STATUS)
#define	OSSTORE_8BIT_DELAYS_IN_STATUS	FALSE
#endif

#if !defined(OSTASK_POINTER_TYPE)
#define OSTASK_POINTER_TYPE
#endif

#if !defined(OSUnprotect)
#define OSUnprotect()
#endif    
                                        
#if !defined(OSUSE_CHAR_SIZED_BITFIELDS)
#define OSUSE_CHAR_SIZED_BITFIELDS      FALSE
#endif
#if ( OSUSE_CHAR_SIZED_BITFIELDS \
  && ( ( OSCOMPILER == OSHT_8051C ) \
  || ( ( OSCOMPILER == OSIAR_ICC ) && ( OSTARGET == OSMSP430 ) ) ) )
#error salvo.h: selected compiler does not support bitfield packing.
#endif

#if !defined(OSUSE_EXTERN_ARRAY_SIZES)        
#define OSUSE_EXTERN_ARRAY_SIZES        FALSE    
#endif    

#if !defined(OSUSE_MEMSET)
#define OSUSE_MEMSET                    FALSE
#endif    

#if !defined(OSUSE_PRINTF_P_FORMAT)
#define OSUSE_PRINTF_P_FORMAT           FALSE
#endif
                                    

#endif /* __SALVOPSH_H */


