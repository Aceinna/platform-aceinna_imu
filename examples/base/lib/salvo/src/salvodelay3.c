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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvodelay3.c,v $
$Author: aek $
$Revision: 3.12 $
$Date: 2008-04-27 14:45:40-07 $

OSGetTS(), OSSetTS(). Read and Write current task's timestamp.

************************************************************/

#include <salvo.h>

#if OSENABLE_DELAYS && OSENABLE_TICKS


/************************************************************
****                                                     ****
**                                                         **
OSGetTSTask(tcbP)

Returns task's timestamp.


**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSGETTSTASK_DELAY3_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
OStypeTS OSGetTSTask( OStypeTcbP tcbP )
OSMONITOR_KEYWORD_POST
{
    OStypeTS timestamp;


    OSEnterCritical();
    OSIncCallDepth();

    timestamp = tcbP->dly.timestamp;

    OSDecCallDepth();
    OSLeaveCritical();

    return timestamp;
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSGETTSTASK_DELAY3_C
#endif


/************************************************************
****                                                     ****
**                                                         **
OSSetTSTask(tcbP, timestamp)

Sets task's timestamp.


**                                                         **
****                                                     ****
************************************************************/
#if OSCALLGRAPH_CONTROL_REQUIRED
#define __OSSETTSTASK_DELAY3_C
#include <salvomcg.h>
#endif

OSMONITOR_KEYWORD_PRE
void OSSetTSTask( OStypeTcbP  tcbP,
                  OStypeTS    timestamp )
OSMONITOR_KEYWORD_POST
{
  OSEnterCritical();
  OSIncCallDepth();

  tcbP->dly.timestamp = timestamp;

  OSDecCallDepth();
  OSLeaveCritical();
}

#if OSCALLGRAPH_CONTROL_REQUIRED
#include <salvoscg.h>
#undef __OSSETTSTASK_DELAY3_C
#endif

#endif /*  #if OSENABLE_DELAYS && OSENABLE_TICKS */

