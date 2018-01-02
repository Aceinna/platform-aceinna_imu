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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvoadc.h,v $
$Author: aek $
$Revision: 3.19 $
$Date: 2008-06-20 11:37:39-07 $

Salvo port-specific header file includes.

************************************************************/
#ifndef __SALVOADC_H
#define __SALVOADC_H

/************************************************************
****                                                     ****
**                                                         **
Auto-detect compiler in use, set OSCOMPILER and OSTARGET
accordingly. Set OSTOOLSET_SUPPORTED to TRUE if supported.

Can be overriden for testing purposes by forcing to OSUNDEF 
or a defined compiler in salvocfg.h.

**                                                         **
****                                                     ****
************************************************************/

#if !defined(OSCOMPILER) && !defined(OSTARGET)

/* GNU C Compiler (gcc) */
#if   defined(__GNUC__)
#undef  OSCOMPILER
#undef  OSTARGET

  #if  defined(__AVR__)
    #define OSTOOLSET_SUPPORTED           TRUE
    #define OSCOMPILER                    OSGCC_AVR
    #define OSTARGET                      OSAVR
    
  #elif defined(__ARM) || defined(__THUMB)
    #define OSCOMPILER                    OSGCC_ARM
    #define OSTARGET                      OSARM7
    
  #elif defined(__thumb2__)
    #define OSCOMPILER                    OSGCC_CM3
    #define OSTARGET       					      OSARMCM3                 
    
  /* Microchip MPLAB C30 compiler */                                        
  #elif defined(C30) || defined(__C30) || defined(__C30__)
	#define OSTOOLSET_SUPPORTED             TRUE
  #define OSCOMPILER                      OSMPLAB_C30
	#if   defined(__PIC24F__)
	#define OSTARGET                        OSPIC24F
	#elif defined(__PIC24H__)
	#define OSTARGET                        OSPIC24H
	#elif defined(__dsPIC30F__)
	#define OSTARGET                        OSDSPIC30F
	#elif defined(__dsPIC33F__)
	#define OSTARGET                        OSDSPIC33F
	#else
	#define OSTARGET                        OSPIC24F // default
	#endif

  /* Microchip MPLAB C32 compiler */                                        
  #elif defined(__PIC32MX__)
	#define OSTOOLSET_SUPPORTED             TRUE
  #define OSCOMPILER                      OSMPLAB_C32
	#define OSTARGET                        OSPIC32

  /* GNU17 for EPSON S1C17 16-bit micros */
  #elif defined(__c17)
	#define OSTOOLSET_SUPPORTED             TRUE
  #define OSCOMPILER                      OSGCC_S1C17
	#define OSTARGET                        OSEPSON_S1C17

  #else
    #define OSCOMPILER                      OSGCC
    #define OSTARGET                        OSIX86
    
  #endif


/* Cosmic compilers -- currently not autodetected -- 	*/
/*  set OSCOMPILER and OSTARGET in salvocfg.h!			*/


/* HI-TECH compilers */
#elif defined(HI_TECH_C)
#undef  OSCOMPILER
#undef  OSTARGET
#if   defined(_PIC12)
#define OSCOMPILER                      OSHT_PICC
#define OSTARGET                        OSPIC12
#elif defined(_PIC14)
#define OSCOMPILER                      OSHT_PICC
#define OSTARGET                        OSPIC16
#elif defined(_PIC16)
#define OSCOMPILER                      OSHT_PICC
#define OSTARGET                        OSPIC17
#elif defined(_PIC18)
#define OSCOMPILER                      OSHT_PICC
#define OSTARGET                        OSPIC18
#elif defined(i8051)
#define OSCOMPILER                      OSHT_8051C
#define OSTARGET                        OSI8051
#elif defined(_V8)
#define OSCOMPILER                      OSHT_V8C
#define OSTARGET                        OSVAV8
#elif defined(__MSP430C__)
#define OSCOMPILER                      OSHT_430C
#define OSTARGET                        OSMSP430
#endif /* if .. elif .. */


/* V8C v7.85PL1 failed to define HI_TECH_C */
#elif !defined(HI_TECH_C) && defined(_V8) 
#error salvo.h: V8C v7.85PL1 or higher required -- aborting.


/* IAR ARM, AVR, MSP430 and PIC18 compilers */                                   
#elif defined(__IAR_SYSTEMS_ICC__) 
#if defined(OSCOMPILER)
#undef  OSCOMPILER
#endif
#if defined(OSTARGET)
#undef  OSTARGET
#endif

#define OSTOOLSET_SUPPORTED             TRUE
#define OSCOMPILER                      OSIAR_ICC
#if   ((__TID__ >> 8) & 0x7F) == 0x31
#define OSTARGET                        OSPIC18
#elif ((__TID__ >> 8) & 0x7F) == 43
#define OSTARGET                        OSMSP430
#elif ((__TID__ >> 8) & 0x7F) == 79
#define OSTARGET                        OSARM7
#elif ((__TID__ >> 8) & 0x7F) == 0x5A
#define OSTARGET                        OSAVR
#else
#error salvo.h: Unidentified IAR compiler.
#endif        
  
/* Keil Cx51 compiler */
#elif defined(__C51__) || defined(__CX51__)
#undef  OSCOMPILER
#undef  OSTARGET
#define OSTOOLSET_SUPPORTED             TRUE
#define OSCOMPILER                      OSKEIL_C51
#define OSTARGET                        OSI8051


/* Keil CARM compiler */
#elif defined(__KEIL__) && defined(__CA__)
#undef  OSCOMPILER
#undef  OSTARGET
#define OSCOMPILER                      OSKEIL_CARM
#define OSTARGET                        OSARM7


/* Metrowerks x86 compiler */
#elif defined(__MWERKS__)
#undef  OSCOMPILER
#undef  OSTARGET
#define OSCOMPILER                      OSMW_CW
#define OSTARGET                        OSIX86


/* Microsoft C/C++ compiler */
#elif defined(_MSC_VER)
#undef  OSCOMPILER
#undef  OSTARGET
#define OSCOMPILER                      OSMSC_CPP
#define OSTARGET                        OSIX86
#define OSTOOLSET_SUPPORTED             TRUE



/* Microchip MPLAB-C18 compiler */                                        
#elif defined(__18CXX)
#undef  OSCOMPILER
#undef  OSTARGET
#define OSCOMPILER                      OSMPLAB_C18
#define OSTARGET                        OSPIC18


/* TASKING CC51 compiler */
#elif defined(_CC51)
#undef  OSCOMPILER
#undef  OSTARGET
#define OSCOMPILER                      OSTASKING_CC51
#define OSTARGET                        OSI8051


/* Archelon / Quadravox 430 compiler */
#elif defined(__ARCHELON__) || defined(__AQCOMPILER__)
#undef  OSCOMPILER
#undef  OSTARGET
#define OSCOMPILER                      OSAQ_430
#define OSTARGET                        OSMSP430


/* ImageCraft compilers */
#elif defined(__IMAGECRAFT__)
#undef  OSCOMPILER
#define OSCOMPILER                      OSIMAGECRAFT
#undef  OSTARGET
#if   defined(_HC11)
#define OSTARGET                        OSM68HC11
#elif defined(_MSP430)
#define OSTARGET                        OSMSP430
#elif defined(_AVR)
#define OSTOOLSET_SUPPORTED             TRUE
#define OSTARGET                        OSAVR
#elif defined(_ARM)
#define OSTARGET                        OSARM7
#else
#error salvo.h: Unknown target for ImageCraft ICC. 
#endif




/* TI's DSP Code Composer for TMS320C24x|28x */
#elif defined(_TMS320C2XX) || defined(__TMS320C28XX__)
#undef  OSCOMPILER
#define OSCOMPILER                      OSTI_DSP_CCS
#undef  OSTARGET
#if   defined(_TMS320C2XX)
#define OSTARGET                        OSTMS320C24X
#elif defined(__TMS320C28XX__)
#define OSTARGET                        OSTMS320C28X
#else
#error salvo.h: Unknown target for TI DSP CCS.
#endif


/* Rowley Associates' CrossWorks MSP430 */
#elif defined(__CROSSWORKS_MSP430)
#undef  OSCOMPILER
#undef  OSTARGET

#define OSTOOLSET_SUPPORTED             TRUE
#define OSCOMPILER                      OSRA_CROSSSTUDIO
#define OSTARGET                        OSMSP430


/* ARM RealView (RVDS) compiler */
#elif defined(__ARMCC_VERSION)
#undef  OSCOMPILER
#define OSCOMPILER                      OSARM_RVDS
#undef  OSTARGET
#if defined(__TARGET_CPU_CORTEX_M3)
#define OSTARGET                        OSARMCM3
#else
#define OSTARGET		   				OSARM7
//#else
//#error salvo.h: Unknown target for ARM RVDS.
#endif



#endif /* #if defined(__GNUC__) #elif ... */

#endif

#endif /* __SALVOADC_H */
