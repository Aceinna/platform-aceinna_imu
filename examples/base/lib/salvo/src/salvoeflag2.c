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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoeflag2.c,v $
$Author: aek $
$Revision: 3.10 $
$Date: 2008-04-27 14:45:39-07 $

Functions to read wait event flags.

************************************************************/

#include <salvo.h>

#if OSENABLE_EVENT_FLAGS


/************************************************************ 
****                                                     ****
**                                                         **
OSReturnEFlag(ecbP)

Used by OSReadEFlag().

Note: Because the OSReturnXyz() funstions all have distinctly
different return types, they are independent of 
OSCOMBINE_EVENT_SERVICES.

**                                                         **
****                                                     ****
************************************************************/
#if OSENABLE_EVENT_READING

#define __OSRETURNEFLAG_EFLAG2_C
#include <salvomcg.h>

OSMONITOR_KEYWORD_PRE
OStypeEFlag OSReturnEFlag( OStypeEcbP ecbP )
OSMONITOR_KEYWORD_POST
OSRETURNEFLAG_ATTR
{
  OStypeEFlag eFlag;
  
  
  OSEnterCritical();
  OSIncCallDepth();
  
  eFlag = ecbP->event.efcbP->eFlag;
          
  OSDecCallDepth();
  OSLeaveCritical();
  
  return eFlag;
}

#include <salvoscg.h>
#undef __OSRETURNEFLAG_EFLAG2_C

#endif 

#endif /* #if OSENABLE_EVENT_FLAGS */



