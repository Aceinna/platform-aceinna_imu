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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvoctx.h,v $
$Author: aek $
$Revision: 3.13 $
$Date: 2008-05-13 11:06:59-07 $

Salvo context-switcher-specific definitions.

************************************************************/
#ifndef __SALVOCTX_H
#define __SALVOCTX_H

#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************
****                                                     ****
**                                                         **
Port-specific context switcher support.

If the context-switcher used is OSCtxSw(), then there's no
need for labels.

If it's done in-line, then labels are required so that the
compiler can reference the "resume at" address.

**                                                         **
****                                                     ****
************************************************************/

#if (OSCTXSW_METHOD == OSVIA_OSCTXSW)

void OSCtxSw( void );
#if !defined(OS_Yield)
#define OS_Yield()                 OSCtxSw()
#endif


#elif (OSCTXSW_METHOD == OSVIA_OSDISPATCH)

void OSDispatch( void );
void OSCtxSw( void );
void OSReturn( void );
#if !defined(OS_Yield)
#define OS_Yield()                 OSCtxSw()
#endif


#elif ((OSCTXSW_METHOD == OSRTNADDR_IS_PARAM) \
   ||  (OSCTXSW_METHOD == OSRTNADDR_IS_VAR))



#elif (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WLABEL)

void OSDispatch( void );
void OSCtxSw( OStypeTFP tFP );


#elif (OSCTXSW_METHOD == OSVIA_OSDISPATCH_WPARAM)

void OSDispatch(unsigned char param);
void OSCtxSw(unsigned char param);

#else

#error salvo.h: OSCTXSW_METHOD not defined.

#endif


/************************************************************
****                                                     ****
**                                                         **
For those context-switchers that save volatile registers
in the tcb, enable such storage in the tcb.

**                                                         **
****                                                     ****
************************************************************/
#if !defined(OSCTXSW_REGS_SAVED)
#define OSCTXSW_REGS_SAVED         	    1
#endif

#if !defined(OSCTXSW_SAVES_REGS)
#define OSCTXSW_SAVES_REGS             	FALSE
#endif

#ifdef __cplusplus
}
#endif
#endif /* __SALVOCTX_H */
