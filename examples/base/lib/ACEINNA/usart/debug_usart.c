/** ***************************************************************************
 * @file   debug_usart.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * usart specific routines to deal with DEBUG serial port
 *  This file provides an abstraction layer, if the processor changes,
 *  this file may have to change but the callers will not.
 *
 * $Revision: 15935 $
 *****************************************************************************/

#include "stm32f2xx.h"
#include "stm32f2xx_conf.h"
#include "boardDefinition.h"
#include "debug_usart.h"
#include "salvodefs.h"
#include "port_def.h" // port_struct, COM_BUF_SIZE
#include "comm_buffers.h"
#include "timer.h"
#include "xbowsp_init.h"  // for gConfiguration in UseExtUSARTGps todo tm20160527 - do better than to need gConfiguration here
#include "driverGPS.h"

port_struct debugPort;
#define PORT_DBG_RX_BUF_SIZE COM_BUF_SIZE // 512
#define PORT_DBG_TX_BUF_SIZE COM_BUF_SIZE

#define _CR    0x0D
#define _LF    0x0A
#define	_BELL  0x07
#define _TAB   0x09
#define _SPACE 0x20
#define _BS    0x08
#define _DEL   0x7F

/** ****************************************************************************
 * @name DebugSerialPutChar
 * @brief Write character out Serial Port.
 * @param [In] character
 * @retval character
 *******************************************************************************/
unsigned char DebugSerialPutChar (unsigned char c)
{ 
  //FIXME if protection is needed it should be a mutex
//    OSDisableHook();
    COM_buf_add(&(debugPort.xmit_buf), (unsigned char*)&c, 1);
//    OSEnableHook();

    USART_ITConfig( DEBUG_USART, USART_IT_TXE, ENABLE );
    return c;
}
/** ****************************************************************************
 * @nameUseExtUSARTGps
 * @brief return which system is using the USART
 * @param
 * @retval false [0] debug, true [1] gps
 *******************************************************************************/
uint8_t UseExtUSARTGps(void)
{
    // Return true is the useGPS bit is set AND the device does not have an
    //   internal GPS
    return (gConfiguration.userBehavior.bit.useGPS && !IsInternalGPS());
}

/** ****************************************************************************
 * @name DebugSerialReadLine
 * @brief Read a line of characters from a serial terminal Will block until a
 *        complete line is read.
 *        Offers terminal-like editing support:
 *                BS(\x08) or DEL(\x7F) delete previous character
 *                TAB(\t) replace by single space
 *                CR(\r) expanded to  CR+LF
 * @param [In] buf - buffer for storing the line
 * @param [In] *index - pointer to current location in the buffer
 * @param [In] len - total length of the buffer
 * @param [Out] *index should be passed in, same as it was before
 * @retval TRUE if _LF is the last character, FALSE otherwise
 *******************************************************************************/
int DebugSerialReadLine(uint8_t  *buf,
					    uint32_t *index,
						uint32_t len)
{
    uint8_t c = 0;

    while ( COM_buf_bytes_available(&(debugPort.rec_buf)) && c != _LF) {
        COM_buf_get(&(debugPort.rec_buf), // get a character
            &c,
            1);
        if (_TAB == c) {
            c = _SPACE;
        }

        /// Handle special character
        switch (c) {
        case _BS:
        case _DEL:
            if (*index > 0) {
                DebugSerialPutChar(_BS);
                DebugSerialPutChar(_SPACE);
                DebugSerialPutChar(_BS);
                (*index)--;
            }
            break;
        case _CR:
            DebugSerialPutChar(c);
            buf[*index] = c;
            (*index)++;
            c = _LF; ///< fall through to _LF
        case _LF:
            DebugSerialPutChar(c);
            buf[*index] = c;
            *index = 0;
            break;
        default:
            /// Only keep printable characters
            if ((c >= 0x20) && (c <= 0x7E)) {
                /// check for room in the buffer, saving two characters for CR+LF
                if (*index < (len - 2)) {
                     buf[*index] = c;
                    (*index)++;
                    DebugSerialPutChar(c);
                } else { /// buffer full, send BELL
                    DebugSerialPutChar(_BELL);
                }
            }
        } //end switch
    } // end while
 return (c == _LF);
}


/** ****************************************************************************
 * @name IsDebugSerialIdle
 * @brief Determines if the serial port is idle, only should be used when
 *        ouputting a large quantity of data as fast as possible
 * @param N/A
 * @retval True if serial port is idle
 *******************************************************************************/
int IsDebugSerialIdle()
{   // reverses the return so empty is true
    return (COM_buf_bytes_available(&(debugPort.xmit_buf) ) == 0) ? 1 : 0;
}



/** ****************************************************************************
 * @name InitSerialCommunication
 * @brief Initialize DEBUG serial communication on USART1
 * @param N/A
 * @retval N/A
 *******************************************************************************/
void InitDebugSerialCommunication( uint32_t baudRate )
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
	static uint8_t    debugPort_rx [PORT_DBG_RX_BUF_SIZE];
    static uint8_t    debugPort_tx [PORT_DBG_TX_BUF_SIZE];

    /// Enable GPIO clock
    RCC_AHB1PeriphClockCmd(DEBUG_USART_TX_GPIO_CLK | DEBUG_USART_RX_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

    /// configure COM1 TX Pins
    GPIO_InitStructure.GPIO_Pin   = DEBUG_USART_TX_PIN; // A9
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,
                     DEBUG_USART_TX_SOURCE,
                     DEBUG_USART_TX_AF);

    /// configure COM1 RX Pins
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN; // A10
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,
                     DEBUG_USART_RX_SOURCE,
                     DEBUG_USART_RX_AF);

    /** USARTx configured as follow:
    - BaudRate = 921600 baud - passed in
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate            = baudRate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(DEBUG_USART, &USART_InitStructure);

    /// initialize software circular buffers comm_buffers.c
    COM_buf_init(&debugPort,
                 debugPort_tx,
                 debugPort_rx,
                 PORT_DBG_RX_BUF_SIZE,
                 PORT_DBG_TX_BUF_SIZE);

    USART_Cmd(DEBUG_USART, ENABLE);
    /// clear anything in the rx
    if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE)) {
        USART_ReceiveData(DEBUG_USART);
    }

    /// now for interrupts
    USART_ITConfig( DEBUG_USART, USART_IT_RXNE, ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQn;
    if ( UseExtUSARTGps() == true) { // using the USART for an external GPS
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x8;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
        setExternalPortPtr(&debugPort);
    } else { // Debug console
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    }
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}

/** ****************************************************************************
 * @name DEBUG_USART_IRQ
 * @brief USART handles rx and tx interrupts
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void DEBUG_USART_IRQ()
{
    uint8_t ch;

//FIXME if protection is needed it should be a mutex
//    OSDisableHook(); /// to disable interrupts

    /// Check the state of the "receive data register not empty" flag. Set binary
    ///   semaphore when detected
    if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE)) {
        ch = DEBUG_USART->DR;
        COM_buf_add(&(debugPort.rec_buf),
                    &ch,
                    1);
        OSSignalBinSem(BINSEM_SERIAL_RX);
    }

    /// Check the state of the "transmit data register empty" flag
    if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE)) {
          if ( IsDebugSerialIdle() ) {
            USART_ITConfig( DEBUG_USART, USART_IT_TXE, DISABLE );
        } else {
            COM_buf_get(&(debugPort.xmit_buf), // get a character
                        &ch,
                        1);
            USART_SendData(DEBUG_USART, ch);
        }
    }
//    OSEnableHook(); // re-enable interrupts
}
