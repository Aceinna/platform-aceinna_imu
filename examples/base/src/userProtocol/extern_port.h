/** ***************************************************************************
 * @file extern_port.h functions for general external port interface
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief Memsic ucb (Unified Code Base) external serial interface
 *        external communication port structures
 *****************************************************************************/
#ifndef EXTERN_PORT_H
#define EXTERN_PORT_H

#include "port_def.h"
#include <stdint.h>
#include "ucb_packet.h"

#define PRIMARY_UCB_PORT 0
typedef uint16_t ExternPortTypeEnum;

extern void   	ExternPortInit         (void);
extern BOOL     HandleUcbRx (ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
extern void     HandleUcbTx (ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);
extern void	 	ExternPortWaitOnTxIdle (void);

#endif
