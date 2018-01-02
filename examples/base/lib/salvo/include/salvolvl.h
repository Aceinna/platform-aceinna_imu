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
places(TM). Copyright (C) 1995-2004 Pumpkin, Inc. and its
Licensor(s). All Rights Reserved.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvolvl.h,v $
$Author: aek $
$Revision: 3.6 $
$Date: 2007-11-09 18:38:05-08 $

pragmas to handle interrupt levels for stackless compilers.

See comments in salvomcg.h on why this header file doesn't
have include guards ...

************************************************************/


/************************************************************
****                                                     ****
**                                                         **
PICC, PICC-Lite, PICC-18 and V8C:

**                                                         **
****                                                     ****
************************************************************/
#if  (OSCOMPILER == OSHT_PICC) || (OSCOMPILER == OSHT_V8C)

#if   (OSINTERRUPT_LEVEL == 0)
#pragma interrupt_level 0
#elif (OSINTERRUPT_LEVEL == 1)
#pragma interrupt_level 1
#elif (OSINTERRUPT_LEVEL == 2)
#pragma interrupt_level 2
#elif (OSINTERRUPT_LEVEL == 3)
#pragma interrupt_level 3
#elif (OSINTERRUPT_LEVEL == 4)
#pragma interrupt_level 4
#elif (OSINTERRUPT_LEVEL == 5)
#pragma interrupt_level 5
#elif (OSINTERRUPT_LEVEL == 6)
#pragma interrupt_level 6
#elif (OSINTERRUPT_LEVEL == 7)
#pragma interrupt_level 7
#else
#error salvolvl.h: Invalid interrupt level: OSINTERRUPT_LEVEL.
#endif

#endif /* #if OSCOMPILER == ... */
