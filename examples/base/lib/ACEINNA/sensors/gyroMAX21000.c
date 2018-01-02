/** ***************************************************************************
 * @file gyroMAX21000.c Gyroscope interface for the Maxim 21000 gyro Some
 *       features MAX21000 of interest for implementation: SPI, up to 10MHz
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
#include <stdint.h>
#include "dmu.h"
#include "boardDefinition.h"
#include "xbowsp_algorithm.h"
#include "configuration.h"
#include "ucb_packet.h"
#include "spi.h"
#include "gyroscope.h"
#include "gyroMAX21000.h"
#include "debug.h"

/** ****************************************************************************
 * @name MAXIM21000WhoAmI
 * @param [out] whoami (0xb1) - buffer to put ID into
 * @retval 1 = succeed 0 = fail
 ******************************************************************************/
uint8_t MAX21000WhoAmI(uint32_t *whoami)
{
    uint8_t rx      = 0;
    uint8_t retries = 3;
    uint8_t sameID = 0;

    spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_128); // go slow  ~0.5  Mhz
    while (retries && rx !=  WHO_AM_I) {
        rx  = _ReadGyroRegister( WHO_AM_I_REG ); // reg 0x20 = 0xB1
        retries--;
    }
    if ( whoami ) {
        *whoami  = rx;
    }
    spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_8); // go fast ~7.5 MHz

    sameID = ( rx == WHO_AM_I );
    return sameID;
}

/** ****************************************************************************
 * @name GyroGetGain return the gain for the device
 * @param N/A
 * @retval gain
 ******************************************************************************/
uint16_t GyroGetGain()
{
    uint8_t config;

    spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_128); // go slow  ~0.5  Mhz

	_WriteGyroRegister(BANK_SEL_REG, BANK_00); // 0x21, 00
	config = _ReadGyroRegister( SENSE_CFG0 ); // 0x00
	config &= FS_MASK;

    spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_8); // go fast ~7.5 MHz

	switch ( config ) {
        case FS_2000_DPS:
            return 15;
        case FS_1000_DPS:
            return 30;
        case FS_500_DPS:
            return 60;
        case FS_250_DPS:
            return 120;
	}
	return TRUE;
}

/** ****************************************************************************
 * @name GyroSelfTest debug CONSOLE test read the serial number print the
 *       results to the debug port then check for whoami - double test for am I
 *       alive?
 * @param N/A
 * @retval pass or error
 ******************************************************************************/
uint8_t  GyroSelfTest()
{
    int      i;
    uint16_t serialNumberWord;
    uint8_t  error;

    error = _WriteGyroRegister(BANK_SEL_REG, BANK_01); // 0x21, 0x0x

    DEBUG_STRING("\r\n\tMAX21000 Serial number: ");
    for (i = 0; i < 6; i+=2) {
        serialNumberWord = _ReadGyroRegister(SERIAL_0_REG + i); // 0x1a + i
        serialNumberWord <<= 8;
        serialNumberWord |= _ReadGyroRegister(SERIAL_0_REG + i + 1);
        DEBUG_HEX(" ", serialNumberWord);
    }
    DEBUG_STRING("\r\n\t");

    // return back to normal
	error = _WriteGyroRegister(BANK_SEL_REG, BANK_00);
    if (error) {
      return FALSE;
    }

    return MAX21000WhoAmI(NULL);
}

/** ****************************************************************************
 * @name GyroSelfTest_Bias This function sends the command to the
 *       rate-sensor to apply or remove an electro-static force to the sensors,
 *       resulting in a sensor bias.
 * @param [in] apply - flag to set or unset the bias
 * @retval N/A
 ******************************************************************************/
void GyroSelfTest_Bias( uint8_t apply )
{
    uint8_t config = 0x00;

    _WriteGyroRegister( BANK_SEL_REG, BANK_00 ); // 0x21(33)
    config = _ReadGyroRegister( SENSE_CFG1 ); // unit configuration

    switch (apply) {
        case APPLY:
            config |= 0x40;   // 0x40 = b01000000 set the self-test bit
            break;
        case REMOVE:
        default:
            config &= 0x3F;   // 0x3F = b00111111 reset the self-test bit
    }
    // send the command
    _WriteGyroRegister( SENSE_CFG1, config ); // 0x01, 0x40
}

/** ****************************************************************************
 * @name GyroConfig initialize the gyro sensor hardware
 * @param [in] range - max Degrees per Second
 * @param [in] outputDataRate - hz
 * @retval status
 ******************************************************************************/
uint8_t MAX21000Config(uint32_t *range,
                   uint32_t *outputDataRate)
{
    uint8_t  config;
    uint8_t  error;
    uint8_t  rangeFlag;
    uint16_t odr;

    /// Reduce SPI clock frequency while configuring rate-sensor (~0.5  Mhz)
    spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_128 );

    /// 0x21(33), 0x00  Select register bank 0x00
    error = _WriteGyroRegister(BANK_SEL_REG, BANK_00);

    /// Select the range setting based on the input argument to this function
    if (*range >= _2000_Degrees_Per_Second) {
        rangeFlag = FS_2000_DPS;
        *range = 2000;
    } else if (*range >= _1000_Degrees_Per_Second) {
        rangeFlag = FS_1000_DPS;
        *range = 1000;
    } else if (*range >= _500_Degrees_Per_Second) {
        rangeFlag = FS_500_DPS;
        *range = 500;
    } else {
        // Default range is 250 dps
        rangeFlag = FS_250_DPS;
        *range = 250;
    }

    /// Set sensor range, place chip in power-down mode, and enable all sensors
    config = rangeFlag << 6 | PW_MODE_PD << 3 | EN_ALL_RATE << 0;
    error += _WriteGyroRegister(SENSE_CFG0, config); // 0x00, 0x37

    /// Set "filter cutoff frequency" used by the Maxim chip (nominal setting: 75 Hz)
    config = 0x00;
    config = SNS_BW_50 << 2 | 0x0 << 0; // 0x30 | 0x00
    error += _WriteGyroRegister(SENSE_CFG1, config); // 0x01, 0x30

    /// Set the ODR based on the input argument to this function
    /// section 8.2.3  of the MAX21000 User's Guide
    if (*outputDataRate >= _100_Hz) {
        /// ODR = 10kHz/(n+1) where n is the config value
        odr = (uint8_t)( (10000 / *outputDataRate) - 1 );  // 10000/277 - 1 = 35
        *outputDataRate = 10000 / (odr + 1) ;
    } else if (*outputDataRate >= _20_Hz) {
        /// ODR = 10kHz/(100+5*(n-99)) where n is the config value
        odr = (2000 / *outputDataRate) + 79;
        *outputDataRate = 2000 /( odr - 79 ) ;
    } else { // < 20 Hz
        odr = (500 / *outputDataRate) + 154;
        if (odr > 0xFF) {
            odr = 0xFF;  // max value
        }
        *outputDataRate = 500 / ( odr - 154 ) ;
    }

    /// Set the Output Data Register
    config = odr;
    error += _WriteGyroRegister(SENSE_CFG2, config); // 0x02, 0x23

    /// Enable the low-pass filter and disable the high-pass filter
    config = 0x0 << 5 | 0x0 << 4;
    error += _WriteGyroRegister(SENSE_CFG3, config); // 0x03, 0x00

    /// Enable DR cleared when all bytes are read, update temperature when both
    ///   bytes are read, and enable the temperature sensor
    config = DATA_RDY_ALL << 4 | 0x0 << 1 | 0x1 << 0;
    error += _WriteGyroRegister(SENSE_CFG3, config); // 0x03, 0x00

    /// Disconnect all pull-ups/downs on the DSYNC and INT pins
    error += _WriteGyroRegister(IO_CFG, 0x00); // 0x03, 0x00

    /// Disable the I2C bus
    config = I2C_ONLY_SPI << 4 | PAD_CURR_6 << 2 | I2C_OFF << 0;
    error += _WriteGyroRegister(I2C_CFG, config); // 0x03, 0x00

    /// Set sensor range, place chip in normal mode, and enable all sensors
    config = rangeFlag << 6 | PW_MODE_NORMAL << 3 | EN_ALL_RATE << 0;
    error += _WriteGyroRegister(SENSE_CFG0, config); // 0x00, 0x37

    // Return the SPI clock frequency to nominal frequency (~7.5  Mhz)
    spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_8 );

    // Upon sucessful initialization, return true if no errors were detected
    return (error == 0);
}

