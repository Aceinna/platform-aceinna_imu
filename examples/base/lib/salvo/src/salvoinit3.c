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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoinit3.c,v $
$Author: aek $
$Revision: 3.6 $
$Date: 2008-04-27 14:45:36-07 $

OSInitTcb()

************************************************************/

#include <salvo.h>

#if OSENABLE_TASKS


/************************************************************
****                                                     ****
**                                                         **
OSInitTcb()

This function is required because HiTech PIC-C's memset()
does not support clearing of RAM in banks 2 & 3 (because
memset's arg is 8 bits). PIC-C v7.85 9/99.

NOTE: *(OStypeCharTcbP) tcbP++ = '\0'; doesn't work, 'cause
the increment is by sizeof(OStypeTcb) and not sizeof(char).

**                                                         **
****                                                     ****
************************************************************/
#if OSCLEAR_GLOBALS
#if !OSUSE_MEMSET

void OSInitTcb( OStypeTcbP tcbP )
{
  OStypeInt8u n;
  OStypeCharTcbP charP;


  n     =     sizeof(OStypeTcb);
  charP = (OStypeCharTcbP) tcbP;

  while(n--) {
    *charP++ = '\0';
  }
}


#endif /* #if !OSUSE_MEMSET */
#endif /* #if OSCLEAR_GLOBALS */

#endif /* #if OSENABLE_TASKS */


