/** ***************************************************************************
 * @file configureGPIO.h BSP call to set up GPIO pins
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef _CONFIGURE_IO_H_
#define _CONFIGURE_IO_H_

#include <stdint.h>

#include "stm32f4xx.h"
#include "boardDefinition.h"


void ResetSTIForNormalMode(void);
void ResetSTIForBootMode(void);


#endif