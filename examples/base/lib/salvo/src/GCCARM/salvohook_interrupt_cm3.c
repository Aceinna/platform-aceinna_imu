/************************************************************ 
Copyright (C) 1995-2004 Pumpkin, Inc. and its
Licensor(s). Freely distributable.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\GCCARM\\salvohook_interrupt_cm3.c,v $
$Author: aek $
$Revision: 3.0 $
$Date: 2006-04-25 16:33:12-07 $

File illustrates user interrupt control functions 
for use with Salvo applications built with the
GCC ARM C compiler for a target in the ARM Cortex-M3
family.

In this implementation, interrupts are controlled globally
and blindly, i.e. without regard to the interrupt control
bit's prior setting.
 
Without disabling the global interrupts during Salvo's 
critical sections, corruption will occur when OSTimer() 
is called from within an IRQ.

Dummy versions of OSDisable|Enable|Restore|SaveInts() 
are included in all Salvo libraries. If a Salvo 
application does not use interrupts, then the user
need not create any application-specific versions of
these functions.

************************************************************/

#include <salvo.h>

/* code to assemble if using GNU tools */

/* Simple global interrupt disable. */
void OSDisableHook(void)
{
    asm volatile
        (
         "cpsid   i\n"
        );
}

/* Simple global interrupt enable. */
void OSEnableHook(void)
{
    asm volatile
        (
         "cpsie   i\n"
        );
}


/* Since we don't care about the previous state of global */
/*  interrupt enables, there's nothing to save ...        */
void OSSaveHook(void)
{
	;
}


/* ... and nothing to restore -- just blindly re-enable	*/
/*  interrupts.											*/
void OSRestoreHook(void)
{
    asm volatile
        (
         "cpsie   i\n"
        );
}

