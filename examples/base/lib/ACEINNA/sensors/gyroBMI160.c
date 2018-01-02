/** ***************************************************************************
 * @file gyroBMI16.c Gyroscope functions for the BMI160 gyro
 * @author
 * @date   March, 2017
 * @copyright (c) 2017 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Note, the gyro should implement the interface described in
 * gyroscope.h. This file just provides the specifics, for use by the
 * associated C file only.
 *****************************************************************************/

#include <stdint.h>
#include "salvodefs.h"
#include "xbowsp_algorithm.h"
#include "configuration.h"
#include "ucb_packet.h"
#include "gyroscope.h"
#include "gyroBMI160.h"
#include "spi.h"
#include "debug.h"
#include "dmu.h"
#include "boardDefinition.h"
#include "timer.h"
// #include "configureGPIO.h" // IO3 (B11) debug timing pin
#include "Indices.h"

#include "taskDataAcquisition.h" // LimitInt16Value()

#include "lowpass_filter.h"

uint8_t isGyroPowerOn(void)
{
  uint8_t bmi160_power_status = 0;

  spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_128); // go slow  ~0.5  Mhz

  bmi160_power_status = _ReadGyroRegister(BMI160_POWER_STATUS_REG_ADDR);

  spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_8); // go fast ~7.5 MHz

  if (((bmi160_power_status & BMI160_GYRO_POWER_MASK) == BMI160_GYRO_NORMAL_POWER))
      return true;

  return false;
}

uint8_t isAccelPowerOff(void)
{
  uint8_t bmi160_power_status = 0;

  spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_128); // go slow  ~0.5  Mhz

  bmi160_power_status = _ReadGyroRegister(BMI160_POWER_STATUS_REG_ADDR);

  spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_8); // go fast ~7.5 MHz

  if (((bmi160_power_status & BMI160_ACCEL_POWER_MASK) == 0))
      return true;

  return false;
}


uint8_t isErrBMI160(void)
{
   uint8_t bmi160_error_status = 0;

   spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_128); // go slow  ~0.5  Mhz

   bmi160_error_status = _ReadGyroRegister(BMI160_ERR_REG_ADDR);

   spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_8); // go fast ~7.5 MHz

   if (!(bmi160_error_status))
     return true;

   return false;
}

uint8_t isBMI160(void)
{
    uint8_t rx      = 0;
    uint8_t retries = 3;
    uint8_t sameID = 0;

    spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_128); // go slow  ~0.5  Mhz

    //
    while (retries && rx !=  BMI160_CHIP_ID) {
        rx  = _ReadGyroRegister( BMI160_CHIP_ID_REG_ADDR );
        retries--;
    }

    spi_set_baud(kGyroSPI,
                 SPI_BaudRatePrescaler_8); // go fast ~7.5 MHz
    
    sameID = ( rx == BMI160_CHIP_ID );
    return sameID;
}

void BMI160Config(uint32_t *range, uint32_t *odr)
{
  GYRO_CONF_REG gyro_conf_reg;
  GYRO_RANGE_REG gyro_range_reg;

  gyro_conf_reg.r = 0;
  gyro_range_reg.r = 0;

  if (*range >= _2000_Degrees_Per_Second) {
        gyro_range_reg.b.gyr_range = 0;
  } else if (*range >= _1000_Degrees_Per_Second) {
        gyro_range_reg.b.gyr_range = 1;
  } else if (*range >= _500_Degrees_Per_Second) {
        gyro_range_reg.b.gyr_range = 2;
  } else if (*range >= _250_Degrees_Per_Second)  {
        gyro_range_reg.b.gyr_range = 3;
  } else {
        // Default range is 125 dps
        gyro_range_reg.b.gyr_range = 4;
  }

  if (*odr >= _3200_Hz) {
      gyro_conf_reg.b.gyr_odr = _3200_Samples_Per_Second;
  } else if (*odr >= _1600_Hz) {
      gyro_conf_reg.b.gyr_odr = _1600_Samples_Per_Second;
  } else if (*odr >= _800_Hz) {
      gyro_conf_reg.b.gyr_odr = _800_Samples_Per_Second;
  } else if (*odr >= _400_Hz) {
      gyro_conf_reg.b.gyr_odr = _400_Samples_Per_Second;
  } else if (*odr >= _200_Hz) {
      gyro_conf_reg.b.gyr_odr = _200_Samples_Per_Second;
  } else if (*odr >= _100_Hz) {
      gyro_conf_reg.b.gyr_odr = _100_Samples_Per_Second;
  } else if (*odr >= _50_Hz) {
      gyro_conf_reg.b.gyr_odr = _50_Samples_Per_Second;
  } else {
      gyro_conf_reg.b.gyr_odr = _25_Samples_Per_Second;
  }
  gyro_conf_reg.b.gyr_bwp = OSR4_mode;

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_128 );

    _WriteGyroRegister(BMI160_GYRO_CONF_REG_ADDR, gyro_conf_reg.r);
    _WriteGyroRegister(BMI160_GYRO_RANGE_REG_ADDR, gyro_range_reg.r);
    DelayMs(100);
  
//  /*
//    // FIXME: Why all the 1 second delays?  Can we remove these?
//  {
//    uint8_t result = 0;
//    //DelayMs(1000);
//
//    result = _ReadGyroRegister(BMI160_GYRO_CONF_REG_ADDR);
//    DelayMs(100);
//
//
//    result = 0;
//    result = _ReadGyroRegister(BMI160_GYRO_RANGE_REG_ADDR);
//    DelayMs(100);
//
//    result = 0;
//    result = _ReadGyroRegister(BMI160_CHIP_ID_REG_ADDR);
//
//    DelayMs(100);
//  }
//    */

   spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_8 );

  return;
}

void powerOnBMI160Gyro(void)
{
  uint8_t bmi160_power = gyro_normal_mode;

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_128 );

  _WriteGyroRegister(BMI160_COMMAND_REG_ADDR, bmi160_power);

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_8 );

  return;
}

void powerOffBMI160Gyro(void)
{
  uint8_t bmi160_power = gyro_suspend_mode;

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_128 );

  _WriteGyroRegister(BMI160_COMMAND_REG_ADDR, bmi160_power);

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_8 );

  return;
}

void powerOnBMI160Accel(void)
{
  uint8_t bmi160_power = accel_normal_mode;

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_128 );

  _WriteGyroRegister(BMI160_COMMAND_REG_ADDR, bmi160_power);

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_8 );

  return;
}

void powerOffBMI160Accel(void)
{
  uint8_t bmi160_power = accel_suspend_mode;

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_128 );

  _WriteGyroRegister(BMI160_COMMAND_REG_ADDR, bmi160_power);

  spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_8 );
  return;
}