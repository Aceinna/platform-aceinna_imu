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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvolicense.c,v $
$Author: aek $
$Revision: 3.11 $
$Date: 2008-04-27 14:45:35-07 $

The purpose of this file is to embed this notice into Salvo
libraries, e.g. the freeware libraries.

************************************************************/
#if !defined(__OSLICENSE_C)
#define __OSLICENSE_C


#include <salvo.h>

#if ( OSCOMPILER != OSKEIL_C51 )
OStypeConstInt8u OSlicense0[] = \
"The contents of this file are subject to the Pumpkin Salvo" \
"License (the \"License\"). You may not use this file except" \
"in compliance with the License. You may obtain a copy of" \
"the License at http://www.pumpkininc.com, or from" \
"warranty(at)pumpkininc.com. Reverse Engineering Prohibited. ";

OStypeConstInt8u OSLicense1[] = \
"Software distributed under the License is distributed on an" \
"\"AS IS\" basis, WITHOUT WARRANTY OF ANY KIND, either express" \
"or implied. See the License for specific language governing" \
"the warranty and the rights and limitations under the" \
"License. ";

OStypeConstInt8u OSLicense2[] = \
"The Original Code is Salvo - The RTOS that runs in tiny" \
"places(TM). Copyright (C) 1995-2008 Pumpkin, Inc. and its" \
"Licensor(s). All Rights Reserved.";
#endif /* #if ( OSCOMPILER != OSKEIL_C51 ) */

#endif /* include guard */

