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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvotid.c,v $
$Author: aek $
$Revision: 3.8 $
$Date: 2008-04-27 14:45:26-07 $

OStID()

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS


/************************************************************
****                                                     ****
**                                                         **
OStID(tcbP, tasks)

Given a pointer to a task, return its task ID (1 ..
OSEVENTS), or 0 on failure.

Note: Supports bounds checking

Note: this function should only used for debugging, as it is
slow.

Returns: taskID or 0
**                                                         **
****                                                     ****
************************************************************/
OStypeID OStID( OStypeTcbP tcbP, OStypeID tasks )
{
  OStypeID i;
  OStypeTcbP localTcbP;


  OSIncCallDepth();
  OSDecCallDepth();

  #if OSENABLE_BOUNDS_CHECKING
  // First, check for validity.
  if ((tcbP < OStcbArea) || (tcbP > OSTCBP(tasks))) {
    OSWarnRtn("OStID",
      OSMakeStr("invalid task control block pointer."),
        (OStypeID) 0);
  }
  #endif
  
  // Then find the eID based on simple address matching.
  localTcbP = OStcbArea;
  for (i = 1; i <= tasks; i++) {
    if (localTcbP == tcbP) {
      return i;
    }
    localTcbP++;
  }
  
  // Lack of address match returns a bad find.
  return 0;
}


#endif /* #OSENABLE_TASKS */

