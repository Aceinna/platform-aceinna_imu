/** ***************************************************************************
 * @file   uart.c
 * @Author jsun
 * @date   2010-12-07 10:43:16 +0800 (Tue, 07 Dec 2010)
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  UART driver for the DMU380's two user serial ports
 *	transmitting and receive of serial data and then number of bytes remaining in
 *	the circular buffer. There is no FIFO on the uart as there is in the 525
 *
 * $Revision: 15150 $
 *****************************************************************************/

#ifndef __UART_H
#define __UART_H
#include "port_def.h"
#include "stm32f2xx.h"

#define USER_COMM_UART kUserA_UART
#define GPS_UART       kUserB_UART

extern void         uart_init(unsigned int uartChannel,uart_hw *port_hw);
extern void         uart_read(unsigned int channel, port_struct *port);
extern void         uart_write(unsigned int channel, port_struct *port);
extern unsigned int bytes_remaining(unsigned int channel, port_struct *port);
extern void         uart_BIT (unsigned int channel);
extern void         uart_IrqOnOff(unsigned int uartChannel, uint16_t uartIrqType, FunctionalState en);

#endif
