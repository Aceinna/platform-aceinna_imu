/** ***************************************************************************
 * @file tempTMP102.cTemperature sensor interface for the TI TMP102 temperature
 * sensor Some features of interest for implementation:
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
 * Temperature sensors interface for the Texas instrument TMP102 thermistor
 * Note, the sensor should implement the interface described in
 * temperatureSensor.h. This file just provides the specifics, for use by the
 * associated C file only.
 * @brief  http://www.ti.com/lit/gpn/tmp102
 *****************************************************************************/
#ifndef TMP102_H
#define TMP102_H

/// I2C address with Addr lines pulled low is             100 1000
/// shifted over to the left by 1, the write address is  1001 0010
//
/// An addrress of 0x92 is used with the development board, for the OEM, the
/// address is 0x90

#define TMP102_I2C_ADDR       0x90

/// From Table 2 in TMP102 datasheet (pointer addresses)
#define TMP102_TEMPERATURE_REG    0
#define TMP102_CONFIG_REG         1

#define TMP102_CONFIG_SHUTDOWN    0x01
#define TMP102_CONFIG_FAULT_MASK  0x18
#define TMP102_CONFIG_FAULT_SHIFT    3
#define TMP102_CONFIG_RES_MASK    0xA0
#define TMP102_CONFIG_RES_SHIFT      5
#define TMP102_CONFIG_RESOLUTION     3 ///< 12 bits (0.0625°C), 320ms for conv

#define TMP102_CONFIG_START_CONV  0xE1

#define TMP102_CONFIG1_DEFAULT    0xA0

#endif /* TMP102_H */
