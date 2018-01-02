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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvohook_interrupt.c,v $
$Author: aek $
$Revision: 3.5 $
$Date: 2008-04-27 14:45:38-07 $

User-defined interrupt control functions. To be replaced by
functions that control interrupts associated with Salvo
services.

************************************************************/

#include <salvo.h>


/************************************************************ 
****                                                     ****
**                                                         **
OSDisableHook()

Disables all sources of interrupts whose ISRs call Salvo
services.

Used in tandem with OSEnableHook().

**                                                         **
****                                                     ****
************************************************************/
void OSDisableHook(void)
{
  ;
}


/************************************************************ 
****                                                     ****
**                                                         **
OSEnableHook()

Enables all sources of interrupts whose ISRs call Salvo
services.

Used in tandem with OSDisableHook().

**                                                         **
****                                                     ****
************************************************************/
void OSEnableHook(void)
{
  ;
}


/************************************************************ 
****                                                     ****
**                                                         **
OSRestoreHook()

Restores interrupt enables for all sources of interrupts 
whose ISRs call Salvo services. Can be restored from a simple
global var, as Salvo's critical sections are not reentrant.

Used in tandem with OSSaveHook().

**                                                         **
****                                                     ****
************************************************************/
void OSRestoreHook(void)
{
  ;
}


/************************************************************ 
****                                                     ****
**                                                         **
OSSaveHook()

Saves interrupt enables for all sources of interrupts 
whose ISRs call Salvo services. Can be saved to a simple
global var, as Salvo's critical sections are not reentrant.

Used in tandem with OSRestoreHook().

**                                                         **
****                                                     ****
************************************************************/
void OSSaveHook(void)
{
  ;
}

