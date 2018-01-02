/** ***************************************************************************
 * @file   taskUserCommunication.h
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * End user communication with the DMU380:
 * - Nav-View (UART) maximum rate of 100 Hz (due to wait in the function below)
 *    handled by the Memsic ucb (Unified Code Base) - handle_packet, send_packet,
 *    extern_port.c, ucb_packet.c comm_buffers.c
 * - SPI communication is an external interrupt driven (asynchronous) bus
 ******************************************************************************/
#ifndef _TASK_USER_COMMUNICATION_H_
#define _TASK_USER_COMMUNICATION_H_
// Define the values that are used to select between SPI and UART communications
#define UART_COMM 0
#define SPI_COMM  1
#define CAN_BUS   2

// Function prototypes
extern void TaskUserCommunication(void);
void        InitBoardConfiguration_Pins(void);

// Define a getter for the communication type (SPI or UART)
extern uint32_t getUserCommunicationType(void);

#endif