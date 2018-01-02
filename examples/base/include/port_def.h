/** ***************************************************************************
 * @file port_def.h communications port data structures used by Uart and
 *                  GPS parsing routines
 * @author tdelong
 * @date   2010-08-27 18:56:33 -0700 (Fri, 27 Aug 2010)
 * @rev 16215
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef PORT_DEF_H
#define PORT_DEF_H

#define COM_BUF_SIZE	512  /// number of bytes in the comm buffers

typedef struct {
    unsigned char 	      *buf_add;	  ///< pointer to (address) comm buffer
    unsigned int 	      buf_size;	  ///< buffer size
    volatile unsigned int buf_inptr;  ///< buffer input pointer - modified in UART ISR
	volatile unsigned int buf_outptr; ///< buffer output pointer - modified in UART ISR
    volatile unsigned int bytes_in_buffer; ///< ammount of buffer used
} cir_buf;

typedef struct{
    unsigned int  baud;  	  ///< com port baud rate index
    unsigned char tx_int_lvl; ///< com port tx FIFO interrupt level
    unsigned char rx_int_lvl; ///< com port rx FIFO interrupt level
    unsigned char tx_int_flg; ///< 0 = transmit int off,1 = transmit int enabled
} uart_hw;

typedef struct{
    unsigned char datatype;
    unsigned int  rec_timeout;
} chan;

typedef struct{
    uart_hw	hw;        	///< UART hardware dependent variables
    chan 	cdef;       ///< COM channel dependent variables
    cir_buf	rec_buf;	///< Receive buffer array of pointers
    cir_buf	xmit_buf;   ///< Transmit buffer array of pointers
} port_struct;

#endif

