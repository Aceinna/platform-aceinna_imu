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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoeid.c,v $
$Author: aek $
$Revision: 3.7 $
$Date: 2008-04-27 14:45:39-07 $

OSeID()

************************************************************/

#include <salvo.h>


/************************************************************
****                                                     ****
**                                                         **
OSeID(ecbP, events)

Given a pointer to an event, return its event ID (1 ..
OSEVENTS), or 0 on failure.

Note: Supports bounds checking

Note: this function should only used for debugging, as it is
slow.

Returns: eventID or 0

**                                                         **
****                                                     ****
************************************************************/
#if OSLOGGING && (OSLOG_MESSAGES > OSLOG_NONE)

#if OSENABLE_EVENTS

OStypeID OSeID( OStypeEcbP ecbP, OStypeID events )
{
  OStypeID i;
  OStypeEcbP localEcbP;


  OSIncCallDepth();
  OSDecCallDepth();

  #if OSENABLE_BOUNDS_CHECKING
  // First, check for validity.
  if ((ecbP < OSecbArea) || (ecbP > OSECBP(events))) {
    OSWarnRtn("OSeID",
      OSMakeStr("invalid event control block pointer."),
        (OStypeID) 0);
  }
  #endif
  
  // Then find the eID based on simple address matching.
  localEcbP = OSecbArea;
  for (i = 1; i <= events; i++) {
    if (localEcbP == ecbP) {
      return i;
    }
    localEcbP++;
  }
  
  // Lack of address match returns a bad find.
  return 0;
}

#endif

#endif /* #if OSLOGGING etc. */



