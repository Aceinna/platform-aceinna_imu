/** ***************************************************************************
 * @file magnetomter.h Magnetometer interface
 * This is a generalized magnetometer interface
 * possibly implemented using the Honeywell HMC5883L magnetometer
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
#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H
#include <stdint.h>
uint8_t InitMagnetometer(); /// this will use boardDefinition.h to set up
                            /// the IO lines returns TRUE if init was successful

#define MAG_RANGE_4000_MILLI_GA 4000
#define MAG_RANGE_4700_MILLI_GA 4700
#define MAG_RANGE_1900_MILLI_GA 1900
#define MAG_RANGE_8100_MILLI_GA 8100

uint8_t MagnetometerConfig(uint32_t *rangeInMilliGauss);

uint8_t MagnetometerStartReading(); /// true if success
uint8_t IsMagnetometerDoneReading(); /// true if read is complete
uint8_t MagnetometerGetLastReading(int16_t *readings); /// true if success
void MagnetomterDataReadyIRQ(void); /// handle mag drdy interrupt


uint8_t  MagnetometerWhoAmI(uint32_t *whoami); /// returns true if value is as expected
uint16_t MagnetometerGetGain();
uint8_t  MagnetometerSelfTest();

#endif /* MAGNETOMETER_H */