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

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvover.h,v $
$Author: aek $
$Revision: 3.15 $
$Date: 2011-06-16 11:07:30-07 $

Salvo version.

************************************************************/
#ifndef __SALVOVER_H
#define __SALVOVER_H

/************************************************************
****                                                     ****
**                                                         **
Version number. Externally (i.e. in an installer's name),
the strings look like:

            Major release:               2.2.0
            Major release, patched:      2.2.0-c
            Beta release:                2.3.0-beta1

**                                                         **
****                                                     ****
************************************************************/
#define OSVER_MAJOR             4
#define OSVER_MINOR             3
#define OSVER_SUBMINOR          0

#define OSVERSION               ( OSVER_MAJOR*100 \
                                + OSVER_MINOR*10  \
                                + OSVER_SUBMINOR )
#define OSVersion()             (OSVERSION)


#endif /* __SALVOVER_H */
