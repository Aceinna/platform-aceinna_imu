/** ***************************************************************************
 * @file   sensor_cal.h
 * @Author Dan
 * @date    2011-02-09 16:52:56 -0800 (Wed, 09 Feb 2011)
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
#ifndef SENSOR_CAL_H
#define SENSOR_CAL_H

#include "sensor.h"

// Function found in calibrate_mag.c
extern uint8_t MagAlign(void);
extern void    InitMagAlignParams(void);

//extern BOOL    MagCalOutOfBounds (void); // used?
#endif