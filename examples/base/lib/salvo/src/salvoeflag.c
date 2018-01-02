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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoeflag.c,v $
$Author: aek $
$Revision: 3.20 $
$Date: 2008-04-27 14:45:40-07 $

Functions to create, set, clear and wait event flags.

************************************************************/

#include <salvo.h>

#if OSENABLE_EVENT_FLAGS

/************************************************************
****                                                     ****
**                                                         **
eFlag services when OSCOMBINE_EVENT_SERVICES is FALSE.

See event.c for fully documented version. The versions
herein should function identically, albeit without reduced
call graphs.

**                                                         **
****                                                     ****
************************************************************/
#if !OSCOMBINE_EVENT_SERVICES

/************************************************************
** OSCreateEFlag(ecbP, efcbP, value)                       **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSCREATEEFLAG_EFLAG_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSCreateEFlag( OStypeEcbP  ecbP,
                         OStypeEfcbP efcbP,
                         OStypeEFlag value )
OSMONITOR_KEYWORD_POST
OSCREATEEFLAG_ATTR
{
  OSEnterCritical();
  OSIncCallDepth();

  ecbP->tcbP               = (OSgltypeTcbP) 0;
  ecbP->event.efcbP        = efcbP;
  #if OSUSE_EVENT_TYPES
  ecbP->type               = OSEV_EFLAG;
  #endif

  ecbP->event.efcbP->eFlag = value;

  OSDecCallDepth();
  OSLeaveCritical();

  return OSNOERR;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSCREATEEFLAG_EFLAG_C
#endif

/************************************************************
** OSWaitEFlag(ecbP, mask, options, timeout)               **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSWAITEFLAG_EFLAG_C
#include <salvomcg.h>
#endif

#if !OSENABLE_TIMEOUTS
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitEFlag( OStypeEcbP   ecbP,
                       OStypeEFlag  mask,
                       OStypeOption options )
OSMONITOR_KEYWORD_POST
#else
OSMONITOR_KEYWORD_PRE
OStypeErr OSWaitEFlag( OStypeEcbP   ecbP,
                       OStypeEFlag  mask,
                       OStypeOption options,
                       OStypeDelay  timeout )
OSMONITOR_KEYWORD_POST
#endif
{
  union {
      OStypeErr     err;
      OStypeBoolean avail;
  } u;
  OStypeEFlag eFlag;


  OSEnterCritical();
  OSIncCallDepth();

  u.avail = FALSE;

  #if OSENABLE_ERROR_CHECKING
  if (!ecbP->event.efcbP) {
    ;
  }
  else
  #endif
  {
    eFlag = ecbP->event.efcbP->eFlag;

    switch (options) {
      case OSANY_BITS:
        if ((eFlag & mask) != 0) {
          u.avail++;
        }
        break;

      case OSEXACT_BITS:
        if ((eFlag ^ mask) == 0) {
          u.avail++;
        }
        break;

      case OSALL_BITS:
        if (((eFlag &= mask) ^ mask) == 0) {
          u.avail++;
        }
        break;

      default:
        break;
        
    }
  }


  #if !OSENABLE_TIMEOUTS
  u.err = OSWaitEvent(ecbP, u.avail);
  #else
  u.err = OSWaitEvent(ecbP, u.avail, timeout);
  #endif

  // Note that no post-OSWaitEvent() ops are required
  //  for event flags because eFlags must be explicitly
  //  cleared by the user after OS_WaitEflag().

  OSDecCallDepth();
  OSLeaveCritical();

  return (u.err);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSWAITEFLAG_EFLAG_C
#endif

/************************************************************
** OSSignalEFlag(ecbP, mask, options)                      **
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSIGNALEFLAG_EFLAG_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeErr OSSignalEFlag( OStypeEcbP   ecbP,
                         OStypeEFlag  mask,
                         OStypeOption options)
OSMONITOR_KEYWORD_POST
OSSIGNALEFLAG_ATTR
{
  union {
      OStypeEFlag eFlag;
      OStypeErr   err;
      OStypeTcbP  tcbP;
  } u;
  OStypeEfcbP efcbP;


  OSEnterCritical();
  OSIncCallDepth();

  #if OSENABLE_BOUNDS_CHECKING
  if ((ecbP < OSECBP(1)) || (ecbP > OSECBP(OSEVENTS_LIMIT))) {
    OSWarn("OSSignalEFlag",
           OSMakeStr("event flag %d nonexistent or invalid.",
                     OSeID(ecbP, OSEVENTS)));

    u.err = OSERR_BAD_P;
  }

  else
  #endif
  {
    #if OSENABLE_ERROR_CHECKING && OSUSE_EVENT_TYPES
    if (ecbP->type != OSEV_EFLAG) {
      OSWarn("OSSignalEFlag",
        OSMakeStr("event %d is not an event flag.",
          OSeID(ecbP, OSEVENTS)));
      u.err = OSERR_EVENT_BAD_TYPE;
    }

    else
    #endif
    {
      // Local handle.
      efcbP = ecbP->event.efcbP;

      #if OSENABLE_ERROR_CHECKING
      if (!efcbP) {
        OSWarn("OSSignalEFlag",
               OSMakeStr("event flag %d does not exist.",
                         OSeID(ecbP, OSEVENTS)));
        u.err = OSERR_EVENT_CB_UNINIT;
      }

      else
      #endif
      {                        
        // Event flags are not affected by fast
        //  or slow signaling.  That's because
        //  all of the tasks waiting the event
        //  flag must analyze the flag and
        //  determine their course of action.
        if ((options & OSSET_EFLAG) != 0) {
          u.eFlag = (OStypeEFlag) (efcbP->eFlag | mask);
        }
        else {
          u.eFlag = (OStypeEFlag) (efcbP->eFlag & ((OStypeEFlag) ~mask));
        }

        // This is the only "filter" -- no
        //  point in launching any tasks if the
        //  eFlag didn't change.
        if (u.eFlag == efcbP->eFlag) {
          OSWarn("OSSignalEFlag",
                 OSMakeStr("no change to event flag %d.",
                           OSeID(ecbP, OSEVENTS)));
          u.err = OSERR_EVENT_FULL;
        }

        // Since it changed, update the flag in
        //  the efcbP and launch all the tasks
        //  that are waiting the eFlag.
        else {
          efcbP->eFlag = u.eFlag;

          if ((options & OSSET_EFLAG) != 0) {

            u.tcbP = ecbP->tcbP;
            while (u.tcbP) {
              OSInsSigQ(u.tcbP, ecbP);
              u.tcbP = ecbP->tcbP;
            }
          }
          u.err = OSNOERR;
        }
      }
    }
  }

  OSDecCallDepth();
  OSLeaveCritical();

  return(u.err);
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSSIGNALEFLAG_EFLAG_C
#endif


#endif /* #if !OSCOMBINE_EVENT_SERVICES */

#endif /* #if OSENABLE_EVENT_FLAGS */



