/**********************************************************
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
places(TM). Copyright (C) 1995-2005 Pumpkin, Inc. and its
Licensor(s). All Rights Reserved.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\GCCARM\\salvoportgccarmcm3.S,v $
$Author: aek $
$Revision: 3.0 $
$Date: 2006-04-25 16:33:01-07 $

Context switcher for GCC ARM C compiler.

Developed using the GNU ARM toolset.

Borowed from Salvo's context switcher for Keil CARM compiler
except for register save and restore (R4-R11 instaed of
R4-R7). R4-R11 corresponds to the ARM standard for register
usage, which GCC ARM follows.

Also modified to use thumb2 instructions where appropriate.  Does
not support ARM, only thumb2.

NOTE: Does not support interwork veneer, due to CM3 only supporting thumb2.

**********************************************************/

                .text
                .extern         OScTcbP
                .extern         OSframeP

/**********************************************************
****                                                   ****
**                                                       **
void OSDispatch (void)

**                                                       **
****                                                   ****
**********************************************************/

/* called from OSSched(), used to transfer control to a task */

                .syntax unified
                .code 16
                .thumb_func
                .global         OSDispatch
OSDispatch:
                /* save LR and working regs, will be restored later
                 * by OSCtxSw (below)
				 */
                STMFD       SP!,{R4-R11,LR} /* thumb2 */
                /* remember what the current stack pointer is for salvo */
				
                MOV         R3,SP
                LDR         R1,=OSframeP
                STR         R3,[R1,#0x0]

                /* get pointer to TCB in R0 (task we are switching to)
                 * and get the task "status" in R1, contains task frame
                 * size in upper 16 bits
                 */
                LDR         R0,=OScTcbP
                LDR         R0,[R0,#0x0]
                LDR         R1,[R0,#0x0]

                /* adjust SP in preparation for switching to the task */

                LSRS        R1,R1,#0x10
                SUBS        R3,R1
                MOV         SP,R3

                /* restore the ARM volatile regs r4-r11 from tcb */
                MOV         R1,R0
                ADDS        R1,#0x8         /* location in TCB for reg save */
                LDMIA       R1,{R4-R11}

                /* get the task's saved instruction pointer
                 * and jump to it
                 */
                LDR         R1,[R0,#0x4]
                BX          R1

                NOP


/**********************************************************
****                                                   ****
**                                                       **
void OSCtxSw ( OStypetFP tFP )

**                                                       **
****                                                   ****
**********************************************************/

                .code 16
                .thumb_func
                .global         OSCtxSw
OSCtxSw:

/* called whenever a task "blocks" */

                /* get a pointer to the TCB for this task, in R0
                 * then save the return address for the task in the TCB
                 */
                LDR         R0,=OScTcbP
                LDR         R0,[R0,#0x0]
                MOV         R1,LR
                STR         R1,[R0,#0x4]

                /* for ARM, we must preserve the volatile regs r4-r11 */
                MOV         R1,R0
                ADDS        R1,#0x8         /* location in TCB for reg store */
                STMIA       R1,{R4-R11}

                /* get the saved salvo SP into R1 and R2 */
				
                LDR         R1,=OSframeP
                LDR         R1,[R1,#0x0]
                MOV         R2,R1

                /* get the current stack pointer (task is using) and
                 * compute the frame size by subtracting from saved
                 * salvo SP
                 */
                MOV         R3,SP
                SUBS        R1,R1,R3

                /* now save the task's computed frame size into the upper
                 * 16 bits of the TCB status field
                 */
                LSLS        R1,R1,#0x10
                LDRH        R3,[R0,#0x0]
                ORRS        R3,R3,R1
                STR         R3,[R0,#0x0]

                /* restore salvo's saved SP
                 * then pop saved regs (saved by OSDispatch-above)
                 * and return, which should be back into OSSched()
                 */
                MOV         SP,R2
                LDMFD       SP!,{R4-R11}
                POP         {R0}
                BX          R0

                NOP
                NOP

    .end

/**********************************************************
****                                                   ****
**                                                       **
End of module
**                                                       **
****                                                   ****
**********************************************************/
