/************************************************************
** Original Copyright (C) 1995-2002 Pumpkin, Inc. and its
** Licensor(s). Freely distributable.
**
** This file changes features availabel from the Salvo RTOS. 
** Note that changing this file is insufficient, you must
** recompile the RTOS (look for an IAR folder in the RtosSalvo
** directory.
************************************************************/

#ifndef SALVOCFG__H
#define SALVOCFG__H

#define OSUSE_LIBRARY               TRUE
#define OSLIBRARY_TYPE              OSL
#define OSLIBRARY_CONFIG            OST

#define OSEVENTS                    25
#define OSEVENT_FLAGS               10
#define OSMESSAGE_QUEUES            2
#define OSTASKS                     10

#define OSTOOLSET_SUPPORTED         TRUE

#endif // SALVOCFG__H
