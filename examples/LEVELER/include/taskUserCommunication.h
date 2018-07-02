/** ***************************************************************************
 * @file   taskUserCommunication.h
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
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef _TASK_USER_COMMUNICATION_H_
#define _TASK_USER_COMMUNICATION_H_
#include "GlobalConstants.h"
// Function prototypes
extern void TaskUserCommunication(void const *argument);
void        InitBoardConfiguration_Pins(void);
extern BOOL UseSpiUserInterface (void);
void        ProcessUserCommands  (void);
void        SendContinuousPacket (int dacqRate);




#endif