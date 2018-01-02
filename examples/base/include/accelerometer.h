/** ***************************************************************************
 * @file   accerometer.h
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * generic accelerometer interface, it should be implemented
 * by whichever accelerometer is in use
 *****************************************************************************/
#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

uint8_t InitAccelerometer(uint8_t); /// use boardDefinition.h to set up
                             /// the IO lines returns TRUE if init was successful
#define ACCEL_RANGE_8G   8
#define ACCEL_RANGE_4G   4
#define ACCEL_RANGE_2G   2

uint8_t AccelerometerConfig(uint32_t *rangeInGs, uint32_t *outputDataRate);

uint8_t AccelerometerStartReading();  // true if success
uint8_t IsAccelerometerDoneReading(); // true if read is complete
uint8_t AccelerometerGetLastReading(int16_t *readings); // true if success
void AccelerometerDataReadyIRQ(void); // handle data ready interrupt

uint8_t  AccelerometerWhoAmI(uint32_t *whoami); // returns true if value is as expected
uint16_t AccelerometerGetGain();

void AccelSelfTest_Bias( uint8_t apply );

/// Getter/setter for gAccel.i2cBusy
uint8_t getAccelI2CBusy( void );
void    setAccelI2CBusy( uint8_t state );

void AccelDataReadyInt( FunctionalState enable );

#define BUSY      1
#define NOT_BUSY  0

#endif /* ACCELEROMETER_H */
