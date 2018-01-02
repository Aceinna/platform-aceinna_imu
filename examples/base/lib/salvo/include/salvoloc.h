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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvoloc.h,v $
$Author: aek $
$Revision: 3.3 $
$Date: 2008-04-27 14:45:22-07 $

Salvo memory type qualifier settings for some compilers
with banked memory architectures.

************************************************************/
#ifndef __SALVOLOC_H
#define __SALVOLOC_H

/************************************************************
****                                                     ****
**                                                         **
Bank specifiers and memory type for some compilers. Used for
objects and pointers to objects.

If OSLOC_ALL is undefined(i.e. user hasn't defined it in
salvocfg.h), it defaults to OSLOC_DEFAULT, defined in
portXyz.h. Else it's set by user in salvocfg.h. Note that
OSLOC_DEFAULT can be blank.

Every undefined OSLOC_XYZ is set to OSLOC_ALL.

This allows the user to specify OSLOC_ALL and then explicitly
set individual ones independently.

MPLAB-C18 doesn't support OSLOC_XYZ because of the need for
explicit sections declared via #pragma udata access SecName.

**                                                         **
****                                                     ****
************************************************************/
#if OSCOMPILER == OSMPLAB_C18
#if OSMPLAB_C18_LOC_ALL_NEAR
#undef  OSLOC_ALL                        
#define OSLOC_ALL                        near
#else
#undef  OSLOC_ALL                        
#define OSLOC_ALL                        
#endif /* #if OSMPLAB_C18_LOC_ALL_NEAR */
#undef  OSLOC_COUNT
#define OSLOC_COUNT                     OSLOC_ALL
#undef  OSLOC_CTCB
#define OSLOC_CTCB                      OSLOC_ALL
#undef  OSLOC_DEPTH
#define OSLOC_DEPTH                     OSLOC_ALL
#undef  OSLOC_ECB
#define OSLOC_ECB                       OSLOC_ALL
#undef  OSLOC_EFCB
#define OSLOC_EFCB                      OSLOC_ALL
#undef  OSLOC_ERR
#define OSLOC_ERR                       OSLOC_ALL
#undef  OSLOC_LOGMSG
#define OSLOC_LOGMSG                    OSLOC_ALL
#undef  OSLOC_LOST_TICK
#define OSLOC_LOST_TICK                 OSLOC_ALL
#undef  OSLOC_MQCB
#define OSLOC_MQCB                      OSLOC_ALL
#undef  OSLOC_MSGQ
#define OSLOC_MSGQ                      OSLOC_ALL
#undef  OSLOC_PS
#define OSLOC_PS                        OSLOC_ALL
#undef  OSLOC_SIGQ
#define OSLOC_SIGQ                      OSLOC_ALL
#undef  OSLOC_TCB
#define OSLOC_TCB                       OSLOC_ALL
#undef  OSLOC_TICK
#define OSLOC_TICK                      OSLOC_ALL
#endif /* #if OSCOMPILER ... */


#if !defined(OSLOC_ALL)
#define OSLOC_ALL                       OSLOC_DEFAULT
#endif
                                        
#if !defined(OSLOC_COUNT)
#define OSLOC_COUNT                     OSLOC_ALL
#endif

#if !defined(OSLOC_CTCB)
#define OSLOC_CTCB                      OSLOC_ALL
#endif

#if !defined(OSLOC_DEPTH)
#define OSLOC_DEPTH                     OSLOC_ALL
#endif

#if !defined(OSLOC_ECB)
#define OSLOC_ECB                       OSLOC_ALL
#endif

#if !defined(OSLOC_EFCB)
#define OSLOC_EFCB                      OSLOC_ALL
#endif

#if !defined(OSLOC_ERR)
#define OSLOC_ERR                       OSLOC_ALL
#endif

#if !defined(OSLOC_LOGMSG)
#define OSLOC_LOGMSG                    OSLOC_ALL
#endif

#if !defined(OSLOC_LOST_TICK)
#define OSLOC_LOST_TICK                 OSLOC_ALL
#endif

#if !defined(OSLOC_MQCB)
#define OSLOC_MQCB                      OSLOC_ALL
#endif

#if !defined(OSLOC_MSGQ)
#define OSLOC_MSGQ                      OSLOC_ALL
#endif

#if !defined(OSLOC_PS)
#define OSLOC_PS                        OSLOC_ALL
#endif

#if !defined(OSLOC_SIGQ)
#define OSLOC_SIGQ                      OSLOC_ALL
#endif

#if !defined(OSLOC_TCB)
#define OSLOC_TCB                       OSLOC_ALL
#endif

#if !defined(OSLOC_TICK)
#define OSLOC_TICK                      OSLOC_ALL
#endif

    
#endif /* __SALVOLOC_H */
