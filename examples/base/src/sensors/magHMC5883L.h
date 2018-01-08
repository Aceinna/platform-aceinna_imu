/** ***************************************************************************
 * @file magHMC5883L.c Magnetometer interface for the Honeywell HMC5883L
 * magnetometer Some features of interest for implementation:
 * Fast 160 Hz Maximum Output Rate
 * Built-In Self Test
 * I2C should run at 400 kHz
 * @author
 * @date   September, 2008
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * The normal method for reading is:
 *  Call MagnetometerStartReading, probably from a timer interrupt
 *  Receive data ready interrupt, this triggers an I2C read
 *  Get I2C read complete flag set in OS (or poll IsMagnetometerDoneReading()
 *  Call MagnetometerGetLastReading to get the data out
 *  Repeat
 *
   the magnetometer should implement the interface described in
 * magnetometer.h. This file just provides the specifics, for use by the
 * associated C file only.
 *****************************************************************************/

#ifndef HMC5883L_H
#define HMC5883L_H

#define HMC5883L_I2C_ADDR      0x3C


#define HMC5883L_CONFIGURATION_A  0
#define HMC5883L_CONFIGURATION_B  1
#define HMC5883L_MODE			  2
#define HMC5883L_X_MSB			  3
#define HMC5883L_X_LSB			  4
#define HMC5883L_Z_MSB			  5
#define HMC5883L_Z_LSB			  6
#define HMC5883L_Y_MSB			  7
#define HMC5883L_Y_LSB			  8
#define HMC5883L_STATUS			  9
#define HMC5883L_ID_A			 10
#define HMC5883L_ID_B			 11
#define HMC5883L_ID_C			 12

#define HMC5883L_ID_A_EXPECTED  'H'
#define HMC5883L_ID_B_EXPECTED  '4'
#define HMC5883L_ID_C_EXPECTED  '3'

#define HMC5883L_STATUS_DRDY      0x01
#define HMC5883L_MODE_READ_SINGLE 0x01 // Single-Measurement Mode, other bits
                                       // must be clear for correct operation

#define HMC5883L_A_TEST_POSITIVE   0x11 // MS0 + bias for self test mode
#define HMC5883L_A_TEST_NEGATIVE   0x12 // MS1 - bias for self test mode
#define HMC5883L_A_TEST_NORMAL     0x10 // MS0 0 MS1 0 no bias

#define HMC5883L_GAIN_SHIFT           5
#define HMC5883L_B_GAIN_0_88_GA    0x00
#define HMC5883L_B_GAIN_1_3_GA     0x01
#define HMC5883L_B_GAIN_1_9_GA     0x02
#define HMC5883L_B_GAIN_2_5_GA     0x03
#define HMC5883L_B_GAIN_4_0_GA     0x04
#define HMC5883L_B_GAIN_4_7_GA     0x05
#define HMC5883L_B_GAIN_5_6_GA     0x06
#define HMC5883L_B_GAIN_8_1_GA     0x07

#endif /* HMC5883L_H */