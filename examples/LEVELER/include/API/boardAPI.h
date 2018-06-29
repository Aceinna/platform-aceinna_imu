/** ***************************************************************************
 * @file boardAPI.h API functions for Interfacing with hardware resources
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
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


#ifndef _BOARD_API_H
#define _BOARD_API_H
#include <stdint.h>

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1
#include "GlobalConstants.h"

void    configureIO3Pin(int mode); // where mode is GPIO_INPUT or GPIO_OUTPUT
void    configureIO3Pin(int mode); // where mode is GPIO_INPUT or GPIO_OUTPUT
void    configureDataReadyPin(int mode);
void    setIO3Pin (BOOL high);
void    setIO2Pin (BOOL high);
void    setDataReadyPin(BOOL high);
uint8_t getIO2PinState();
uint8_t getIO3PinState();
uint8_t getDataReadyPinState();
uint8_t ReadUnitHwConfiguration( void );


#endif
