/** ******************************************************************************
 * @file boardAPI.h API functions for Interfacing with built-in-test functionality
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

#ifndef _BIT_API_H
#define _BIT_API_H

#include <stdint.h>

/** ****************************************************************************
 * @name updateBITandStatus processes BIT and Status word logic.
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void updateBITandStatus();

/** ****************************************************************************
 * @name handleOverRange API flags the sensor over-range status and bit. After
 *       quasi-static condition is met (rates<threshold) then an algorithm
 *       restart is performed if user enabled.
 * @author Darren Liccardo -  Jan. 2006
 * @param N/A
 * @retval TRUE if sensors data is over range
 ******************************************************************************/
BOOL handleOverRange(void);


#endif
