/** ***************************************************************************
 * @file debug_usart.h usart specific routines to deal with debug port
 * @Author
 * @date   September, 2008
 * @rev 15935
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef DEBUG_USART_H
#define DEBUG_USART_H
#include <stdint.h>
#include <stm32f2xx.h>   // For definition of 'FunctionalState'

extern unsigned char  DebugSerialPutChar (unsigned char c);
extern int  IsDebugSerialIdle();
extern int  DebugSerialReadLine(uint8_t *buf, uint32_t *index, uint32_t len);
extern void          InitDebugSerialCommunication( uint32_t baudRate );
extern uint8_t       UseExtUSARTGps( void );

#endif /*DEBUG_USART_H */

