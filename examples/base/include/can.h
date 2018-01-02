/** ***************************************************************************
 * @file can.h control area network functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef __CAN_H
#define __CAN_H
#include <stdint.h>

#include "stm32f2xx.h"
#include "stm32f2xx_can.h"

#define CAN_ERROR                          -1
#define CAN_NO_ERROR                        1

#define MEMSIC_CAN_IDE                      1
#define MEMSIC_CAN_RTR                      0

#define MEMSIC_CAN_BAUD_RATE_RETRY          12

extern void InitCommunication_UserCAN(void);
extern void _CAN_Configure(void (*callback1)(void), void(*callback2)(void));
extern void TaskCANCommunication(void);

#endif
