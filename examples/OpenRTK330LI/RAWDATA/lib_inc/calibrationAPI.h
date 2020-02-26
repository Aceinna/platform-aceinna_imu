/** ***************************************************************************
 * @file calibration.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef __CALIBRATION_API_H
#define __CALIBRATION_API_H
#include <stdint.h>

void     InitFactoryCalibration(void);
void     ApplyFactoryCalibration(void);
uint32_t GetUnitSerialNum();
uint8_t *GetUnitVersion();
int      CalibrationTableValid(int idx) ;
uint16_t GetProductConfiguration();

#endif

