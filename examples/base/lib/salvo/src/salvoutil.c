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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoutil.c,v $
$Author: aek $
$Revision: 3.15 $
$Date: 2008-04-27 14:45:25-07 $

OSSaveRtnAddr()

************************************************************/

#include <salvo.h>


/************************************************************
****                                                     ****
**                                                         **
OSSaveRtnAddr(tFP)

This function is used by in-line context switchers in order 
to cut down on code size.  

NOTE: May be discarded in Salvo 4.

**                                                         **
****                                                     ****
************************************************************/
#if (OSCTXSW_METHOD == OSRTNADDR_IS_PARAM)


#define __OSSAVERTNADDR_UTIL_C
#include <salvomcg.h>

OSMONITOR_KEYWORD_PRE 
void OSSaveRtnAddr( OStypeTFP tFP )
OSMONITOR_KEYWORD_POST
{
  OSEnterCritical();
  OSIncCallDepth();
  
  #if OSUSE_CUSTOM_TFP_FIELD
  OScTcbP->u3.rawTFP = OSDethunkTFP(tFP);
  #else
  OScTcbP->u3.tFP = tFP;
  #endif
  
  OSDecCallDepth();
  OSLeaveCritical();
}

#include <salvoscg.h>
#undef __OSSAVERTNADDR_UTIL_C

#endif

