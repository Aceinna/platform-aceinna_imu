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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvocri.h,v $
$Author: aek $
$Revision: 3.7 $
$Date: 2008-04-27 14:45:24-07 $

Salvo critical-section macros.

************************************************************/
#ifndef __SALVOCRI_H
#define __SALVOCRI_H

/************************************************************
****                                                     ****
**                                                         **
OSEnter|LeaveCritical():

Macros for entering and exiting critical regions of code.

These macros are normally mapped to user hooks.

**                                                         **
****                                                     ****
************************************************************/

// FIXME: static int gIsrDisableCount = 0;

// Note: the interrupt enable/disable commands were moved here to rectify the problem caused by the
//       OS calling OSEnableHook() and OSDisableHook a significant number of times.  This causing
//       the sensor interrupts to not play well together.  See salvohook_interrupt.c for more
//       information.  This doesn't seem to help much.  I get more drop-outs with this in place than
//       with the original methodology.

/*
 * All Salvo services with critical sections begin the
 * critical section with OSEnterCritical().
 */

#include "boardDefinition.h"

/* GPIO_SetBits( DATA_READY_PORT, DATA_READY_PIN ); \
*
*/

#if !defined(OSEnterCritical)
#define OSEnterCritical()          		do { OSDisableHook(); \
                                     	  } while (0);
#endif

//#if !defined(OSEnterCritical)
//#define OSEnterCritical()          		do { \
//                                             gIsrDisableCount++; \
//                                             __ASM  ("cpsid i"); \
//                                           } while (0);
//#endif


/*GPIO_ResetBits( DATA_READY_PORT, DATA_READY_PIN ); \
 * All Salvo services with critical sections end the
 * critical section with OSLeaveCritical().
 */
#if !defined(OSLeaveCritical)
#define OSLeaveCritical()          		do { OSEnableHook(); \
                                        } while (0);
#endif

//#if !defined(OSLeaveCritical)
//#define OSLeaveCritical()          		do { \
//                                             gIsrDisableCount--; \
//                                             if (gIsrDisableCount == 0) { \
//                                                 __ASM  ("cpsie i"); \
//                                             } \
//                                        } while (0);
//#endif


#endif /* __SALVOCRI_H */
