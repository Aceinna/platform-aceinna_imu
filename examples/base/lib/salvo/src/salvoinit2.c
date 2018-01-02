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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoinit2.c,v $
$Author: aek $
$Revision: 3.39 $
$Date: 2008-04-27 14:45:37-07 $

OSCreateTask()
OSInitPrioTask()

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS


/************************************************************
****                                                     ****
**                                                         **
OSCreateTask(taskFP, tcbP, priority)

Initialize a new task by assigning it a tcb,
with the desired priority and the tasks's start address.
If the high bit (OSDONT_START_TASK) of the specified priority
is cleared (i.e. 0), the task will start automatically.
Otherwise use OSStartTask().

The priority argument must be passed, even in array mode.

The tcb pointer is verified, and the priority is
bounds-checked if in queue mode.

Returns: OSNOERR if task is created
         OSERR_BAD_P if specified tcbP appears to be invalid
**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSCREATETASK_INITTASK_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSCreateTask( OStypeTFP  tFP,
                        OStypeTcbP tcbP,
                        OStypePrio prio )
OSMONITOR_KEYWORD_POST
{
  // Punt if tcbP is clearly bad.
  #if OSENABLE_BOUNDS_CHECKING
  if ((tcbP < OSTCBP(1)) || (tcbP > OSTCBP(OSTASKS_LIMIT))) {
    OSWarnRtn("OSCreateTask",
              OSMakeStr("task %d nonexistent or invalid.",
                        OStID(tcbP, OSTASKS)), 
              (OStypeErr) OSERR_BAD_P);
  }

  // v7.87 PL2 and earlier PICC compilers have a
  //  PIC12 bug that shows up if the bounds checking
  //  above is disabled. This is a simple fix.
  //  Problem happens when generating libraries.
  #elif OSWORKAROUND_HT_PICC_1
  if (tcbP == (OStypeTcbP) 0) {
    OSWarnRtn("OSCreateTask",
              OSMakeStr("task %d nonexistent or invalid.",
                        OStID(tcbP, OSTASKS)), 
              (OStypeErr) OSERR_BAD_P);
  }
  #endif

  OSEnterCritical();
  OSIncCallDepth();

  // frame size must always be reset to 0 for each
  //  new task. frameSize is application dependent,
  //  and usually does not include the calling fn's
  //  return address.
  // doing this first is helpful when debugging a
  //  compiler's proper implementation of bitfields.
  #if ((OSCTXSW_METHOD == OSVIA_OSDISPATCH) \
   || (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WLABEL) \
   || (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WPARAM))
  tcbP->status.bits.frameSize = 0;
  #endif

  // If priorities are in use, set up the task's
  //  priority properly.
  #if !OSDISABLE_TASK_PRIORITIES

  // Set priority. Not required if we're using arrays.
  // an explicit cast is required because of the
  //  promotion that occurs due to the AND op.
  #if OSENABLE_ERROR_CHECKING
  OSInitPrioTask(tcbP, (OStypePrio) (prio & ((OStypePrio) ~OSDONT_START_TASK)));
  #else
  tcbP->status.bits.prio = (OStypePrio) (prio & ((OStypePrio) ~OSDONT_START_TASK));
  #endif

  // If priorities are not in use, set all tasks
  //  priorities the same so that they all round-
  //  robin at the same priority level.
  #else
  tcbP->status.bits.prio = OSLOWEST_PRIO;
  #endif

  // Set the remaining status fields.
  tcbP->status.bits.state   = OSTCB_TASK_STOPPED;
  tcbP->status.bits.yielded = FALSE;

  // Initialize task entry point.
  #if OSUSE_CUSTOM_TFP_FIELD
  tcbP->u3.rawTFP = OSDethunkTFP(tFP);
  #else
  tcbP->u3.tFP = tFP;
  #endif

  // PowerC context switcher requires that this field
  //  be initialized.
  #if OSCOMPILER == OSMIX_PC
  tcbP->ctxSwVect = 0;
  #endif

  // If ticks are enabled, we should timestamp the
  //  task so that if the user is using OS_DelayTS()
  //  and forgets to call OSSyncTS() it won't
  //  be too bad.
  #if OSENABLE_DELAYS && OSENABLE_TICKS
  tcbP->dly.timestamp = (OStypeTS) OStimerTicks;
  #endif

  // If OSDONT_START_TASK isn't used, then go ahead
  //  and fire it up!
  if ((prio & OSDONT_START_TASK) == 0) {
    OSInsElig(tcbP);
  }

  OSDecCallDepth();
  OSLeaveCritical();
  OSMsgRtn("OSCreateTask",
           OSMakeStr("task %d created.", OStID(tcbP, OSTASKS)),
           (OStypeErr) OSNOERR);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSCREATETASK_INITTASK_C
#endif

/************************************************************
****                                                     ****
**                                                         **
OSInitPrioTask(tcbP, prio)

Check if specified priority is within bounds, and force it
to be within bounds if it's out-of-bounds.

Note: that the comparison is valid because priorities are
unsigned.

Returns: OSERR if priority is out-of-range
         OSNOERR if priority is valid

**                                                         **
****                                                     ****
************************************************************/
#if !OSUSE_ARRAYS && !OSDISABLE_TASK_PRIORITIES
OStypeErr OSInitPrioTask( OStypeTcbP tcbP,
                          OStypePrio prio )
{
  OStypeErr err;

  OSIncCallDepth();

  if (prio > OSLOWEST_PRIO) {
    prio = OSLOWEST_PRIO;
    OSWarn("OSInitPrioTask",
           OSMakeStr("incorrect task priority, (re-)set to %d.",
                     OSLOWEST_PRIO));

    err = OSERR;
  }
  else {
    OSMsg("OSInitPrioTask",
          OSMakeStr("task priority (re-)set to %d.", prio));

    err = OSNOERR;
  }

  OSDecCallDepth();

  #if (OSCOMPILER == OSMPLAB_C18) && OSMPLAB_C18_LOC_ALL_NEAR
  // MPLAB-C18 chokes on above when using the near qualifier.
  tcbP->status.value &= 0xFFF0;
  tcbP->status.value |= prio;
  #elif OSWORKAROUND_HT_V8C_2
  // V8C generates bizarre code without this fix.
  tcbP->status.value &= 0xF0;
  tcbP->status.value |= prio;
  #else
  tcbP->status.bits.prio = prio;
  #endif

  return err;
}
#endif


#endif /* #if OSENABLE_TASKS */
