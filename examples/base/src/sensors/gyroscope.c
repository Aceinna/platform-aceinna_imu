/** ***************************************************************************
 * @file gyroscope.c Generic functions for gyroscope
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
#include "stm32f2xx_conf.h"
#include "salvodefs.h"
#include "xbowsp_algorithm.h"
#include "configuration.h"
#include "ucb_packet.h"
#include "gyroscope.h"
#include "spi.h"
#include "debug.h"
#include "dmu.h"
#include "boardDefinition.h"
#include "timer.h"
#include "Indices.h"
#include "taskDataAcquisition.h" // LimitInt16Value()
#include "lowpass_filter.h"

#include "gyroMAX21000.h"
#include "gyroBMI160.h"

#include "CompilerFlags.h"    // for GYRO_MAXIM21000 and GYRO_BMI160

#ifdef GYRO_BMI160
static uint8_t BMI160DataBuffer[GYRO_MAX_CACHED_NUM][BMI160_BUFFER_SIZE];
#else
static uint8_t MAX21000DataBuffer[GYRO_MAX_CACHED_NUM][MAX21000_BUFFER_SIZE];
#endif

/** ****************************************************************************
 * @name: _ReadGyroRegister LOCAL
 * @param [in] address - SPI adddress
 * @retval rx - data read at the address
 ******************************************************************************/
uint8_t _ReadGyroRegister(uint8_t address)
{
    uint8_t rx[2];

    address |= READ_INDICATION;

    rx[0] = address;
    GPIO_ResetBits(GYRO_SELECT_PORT, // A4
                   GYRO_SELECT_PIN);
    spi1_transfer(rx,
                  rx,
                  sizeof(rx));
    while (!get_spi1_complete())
    {/* spin */;}
    GPIO_SetBits(GYRO_SELECT_PORT,
                 GYRO_SELECT_PIN);
    return rx[1];
}

/** ****************************************************************************
 * @name: _WriteGyroRegister LOCAL
 * @param [in] address - SPI address
 * @param [in] data - data to write out
 * @retval always return 0
 ******************************************************************************/
uint8_t _WriteGyroRegister(uint8_t address,
                                      uint8_t data)
{
    uint8_t tx[2];

    tx[0] = address;
    tx[1] = data;

    GPIO_ResetBits(GYRO_SELECT_PORT, // A4
                   GYRO_SELECT_PIN);
    spi1_transfer( tx,
                   tx,
                   sizeof(tx));
    while (!get_spi1_complete())
    {/* spin */;}
    GPIO_SetBits(GYRO_SELECT_PORT,
                 GYRO_SELECT_PIN);
    return 0;
}

/** ****************************************************************************
 * @name IsGyroDoneReading get the read complete status for the gyro
 * @param N/A
 * @retval N/A
 ******************************************************************************/
uint8_t IsGyroDoneReading()
{
    OStypeEFlag eFlag = OSReadEFlag( EFLAGS_DATA_READY );
    return eFlag & EF_DATA_GYRO_READY; // 0x01
}

/** ****************************************************************************
 * @name _GyroDMADoneCallback LOCAL Deselect the rate sensor by setting the
 *      line high and indicate via the EFLAG that the rate-sensor data is ready
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _GyroDMADoneCallback()
{
    GYRO_SELECT_PORT->BSRRL = GYRO_SELECT_PIN; // A4

    // Set IO3 low when done obtaining data from the rate-sensor
    GPIOB->BSRRH = GPIO_Pin_11;
}


/** ****************************************************************************
 * @name: BeginRateSensorRead  Called upon time-out of TIM5 or appropriate cycle of
 *                             external sync (replaces GyroDataReadyIRQ in new
 *                             data sampling methodology)
 * function.
 *   Selects the rate sensor chip (set line LOW)
 *   Initiates a read of data from the rate-sensor (save in gGyroDataBuffer)
 * @param N/A
 * @retval status
 ******************************************************************************/
#define  ROLLOVER_VALUE  200
void BeginRateSensorRead()
{
#ifdef GYRO_BMI160
    static uint16_t readCntr = ROLLOVER_VALUE;
#endif

    //
    if (gRate.status >= GYRO_BUFFER_FULL) {
        gRate.status = GYRO_BUFFER_OVERFLOW;
        return;
    }

    /// Select the Rate-Sensor for communication by setting nSS low then start the transfer over
    ///   SPI (DMA finishes the read then executes its callback to deselect the RS chip)
    GYRO_SELECT_PORT->BSRRH = GYRO_SELECT_PIN; /// chip-select - low

#ifdef GYRO_BMI160
    BMI160DataBuffer[gRate.cached_num][0] = BMI160_GYRO_DATA_ADDR | READ_INDICATION;

    readCntr++;
    if( readCntr >= ROLLOVER_VALUE ) {
        // Read the temperature sensor data (by reading the complete set of data
        //   buffers) once every 200 times this code is called.  Otherwise, just
        //   read the rate data (else part of statement).
        readCntr = 0;

        spi1_transfer( &(BMI160DataBuffer[gRate.cached_num][0]),
                       &(BMI160DataBuffer[gRate.cached_num][0]),
                       BMI160_BUFFER_SIZE);
        gRate.temp_buff_num = gRate.cached_num;
    } else {
        spi1_transfer( &(BMI160DataBuffer[gRate.cached_num][0]),
                       &(BMI160DataBuffer[gRate.cached_num][0]),
                       7);
    }
#else
    // Generate the read command by appending the read bit to the register address
    MAX21000DataBuffer[gRate.cached_num][0] = GYRO_X_MSB_REG | READ_INDICATION; // 0x23 | 0x80 = 0xA3
    spi1_transfer( &(MAX21000DataBuffer[gRate.cached_num][0]),
                   &(MAX21000DataBuffer[gRate.cached_num][0]),
                   MAX21000_BUFFER_SIZE);
#endif

    if (gRate.cached_num < (GYRO_MAX_CACHED_NUM - 1)) {
        gRate.cached_num++;
        gRate.status = GYRO_BUFFER_AVAILABlE;
    } else {
        gRate.status = GYRO_BUFFER_FULL;
    }
}


/** ****************************************************************************
 * @name GyroGetLastReading get the data saturate it and transform it to
 * DMU380 coordinate frame
 * @param N/A
 * @retval N/A
 ******************************************************************************/
uint8_t GyroGetLastReading(int16_t *readings)
{
    static uint8_t initFlag = 1;

    //
    uint8_t *data;
    int i;
    static int32_t  gyroFilteredVal[NUM_AXIS];
    uint8_t count, cutoff_freq;
    static uint8_t input_data_rate;

    //
    if( initFlag ) {
        initFlag = 0;
        if( UcbGetSysType() <= UNAIDED_AHRS_SYS ) {
            input_data_rate = BWF_LOWPASS_DATA_RATE_800;
        } else {
            input_data_rate = BWF_LOWPASS_DATA_RATE_400;
        }
    }

    count = gRate.cached_num;

    if (count == 0) {
      return 0;
    }

    // 
    if( readings ) {
        for (i = 0; i < count; i++) {
            /// Works with Nav-View (signal flatlines as opposed to flipping signs)
            /// Limit the rate-sensor reading to prevent the transformation from
            /// causing the signal to saturate with an opposite sign if a
            /// transformation between frames is necessary (can cause 32767 -->
            /// -32768 after transformation)
            /// Transform the data into the DMU380 frame
#ifdef GYRO_BMI160
            data = &(BMI160DataBuffer[i][1]);
            gRate.gyroData[i][X_AXIS] = (data[1] << 8) | data[0];
            gRate.gyroData[i][Y_AXIS] = (data[3] << 8) | data[2];
            gRate.gyroData[i][Z_AXIS] = (data[5] << 8) | data[4];
#else
            data = &(MAX21000DataBuffer[i][1]);
            gRate.gyroData[i][X_AXIS] = (data[0] << 8) | data[1];
            gRate.gyroData[i][Y_AXIS] = (data[2] << 8) | data[3];
            gRate.gyroData[i][Z_AXIS] = (data[4] << 8) | data[5];
#endif

            // Filter the data (or not)
            switch( gConfiguration.analogFilterClocks[0] & 0xF ) {
                case 0x0:
                case 0x8:
                    // Do not prefilter the sensor data
                    gyroFilteredVal[X_AXIS] = gRate.gyroData[i][X_AXIS];
                    gyroFilteredVal[Y_AXIS] = gRate.gyroData[i][Y_AXIS];
                    gyroFilteredVal[Z_AXIS] = gRate.gyroData[i][Z_AXIS];
                    break;

                case 0x1:  // Future values for 1st-order filters from 0x1 to 0x7
                    // 3rd-order, 50 Hz BWF (cascaded 1st-order filters)
                    _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(X_AXIS, gRate.gyroData[i][X_AXIS],
                                                                   &gyroFilteredVal[X_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(Y_AXIS, gRate.gyroData[i][Y_AXIS],
                                                                   &gyroFilteredVal[Y_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(Z_AXIS, gRate.gyroData[i][Z_AXIS],
                                                                   &gyroFilteredVal[Z_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    break;
  
                case 0x9:   // Future values for 2nd-order filters from 0x9 to 0xF (9 to 15)
                    // 4th-order, 50 Hz BWF (cascaded 2nd-order filters)
                    _rateFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(X_AXIS, gRate.gyroData[i][X_AXIS],
                                                                   &gyroFilteredVal[X_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    _rateFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(Y_AXIS, gRate.gyroData[i][Y_AXIS],
                                                                   &gyroFilteredVal[Y_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    _rateFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(Z_AXIS, gRate.gyroData[i][Z_AXIS],
                                                                   &gyroFilteredVal[Z_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    break;

                default:
                    // 3rd-order, 50 Hz BWF (cascaded 1st-order filters)
                    _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(X_AXIS, gRate.gyroData[i][X_AXIS],
                                                                   &gyroFilteredVal[X_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(Y_AXIS, gRate.gyroData[i][Y_AXIS],
                                                                   &gyroFilteredVal[Y_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(Z_AXIS, gRate.gyroData[i][Z_AXIS],
                                                                   &gyroFilteredVal[Z_AXIS],
                                                                   cutoff_freq, input_data_rate);
                    break;
            }
        }

        // Limit the value and adjust the sign to conform with the axis
        //   definition for the system
        
        readings[0] =  LimitInt16Value( gyroFilteredVal[X_AXIS], INT16_LIMIT );
        readings[1] = -LimitInt16Value( gyroFilteredVal[Y_AXIS], INT16_LIMIT );
        readings[2] = -LimitInt16Value( gyroFilteredVal[Z_AXIS], INT16_LIMIT );

        //
        gRate.cached_num = 0;
        gRate.status = GYRO_BUFFER_AVAILABlE;
    }
    return 0;
}

/** ****************************************************************************
 * @name GyroTempGetLastReading return  the MAX-2100 temperature
 * @param [out] reading - the return buffer
 * @retval always return 0
 ******************************************************************************/
uint8_t GyroTempGetLastReading(int16_t* reading)
{
    // Recast the BMI output into the Maxim value.  This permits all other functions
    //   (related to temperature) to remain the same.  Note: if another sensor is
    //   added, the same process must be implemented.
    //
    // Tmax [degC] = ( X [cnts] - 6400 [cnts] ) * ( 1/256 [degC/cnt] ) + 25 [degC]
    // Tbmi [degC] = ( Y [cnts] ) * ( 1/512  [degC/cnt] ) + 23 [degC]
    //
    // X = Y/2 + 5888 (BMI counts expressed as Maxim counts)
    //

    uint8_t * data;
#ifdef GYRO_BMI160
    data = &( BMI160DataBuffer[gRate.temp_buff_num][21] );

    if (reading) {
        int16_t tmp = (data[1] << 8) | data[0];
        reading[0] = (tmp >> 1) + 5888;
    }
#else
    data = &( MAX21000DataBuffer[gRate.cached_num][7] );

    if (reading) {
        reading[0] = (data[0] << 8) | data[1];
    }
#endif
    return 0;
}


/** ****************************************************************************
 * @name GyroTempGetTemperature return  the MAX-2100 temperature
 * @brief C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
 * @param [in] reading - the raw temperature counts
 * @param [out] out- the return buffer in degrees
 * @retval always return 0
 ******************************************************************************/
uint8_t GyroTempGetTemperature(int16_t reading,
                               float   *outTemperature)
{
    *outTemperature = (float) reading / 256.0f;
    return 0;
}

/** ****************************************************************************
 * @name InitGyro set up the SPI bus and synch pin C5
 * @param [in] rangeInDps - max Degrees per Second
 * @param [in] outputDataRate - hz
 * @retval status
 ******************************************************************************/
uint8_t InitGyro()
{
    uint8_t          error;
    GPIO_InitTypeDef GPIO_InitStructure;

    error = spi_configure( kGyroSPI,
                           SPI_CPOL_AND_CPHA_HIGH,
                           &_GyroDMADoneCallback );

    if (error) {
        return error;
    }

    // Reduce SPI clock frequency while initializing rate-sensor (~0.5  Mhz)
    spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_128);

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd(GYRO_SELECT_CLK, ENABLE);

    // Set up chip-select pin (A4) with properties defined for sync pin
    GPIO_InitStructure.GPIO_Pin   = GYRO_SELECT_PIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MAXGYRO_SELECT_PORT, &GPIO_InitStructure);
    GPIO_SetBits(GYRO_SELECT_PORT, GYRO_SELECT_PIN);

    // Return the SPI clock frequency to nominal frequency (~7.5  Mhz)
    spi_set_baud( kGyroSPI,
                  SPI_BaudRatePrescaler_8);

    // (FIXME) JSM: Added for Feng's LPF implementation
    gRate.status = GYRO_BUFFER_IDLE;
    gRate.cached_num = 0;
    gRate.temp_buff_num = 0;

#ifdef GYRO_BMI160
    return isBMI160();
#else
    // Upon sucessful initialization, return the rate-sensor ID
    return MAX21000WhoAmI(NULL);
#endif

}

uint8_t GyroConfig(uint32_t * range, uint32_t * odr)
{
#ifdef GYRO_BMI160
  BMI160Config(range, odr);
  DelayMs(10);

  powerOffBMI160Accel();
  DelayMs(20);

  powerOnBMI160Gyro();
  //DelayMs(200);
  DelayMs(100);
#else
  MAX21000Config(range, odr);
#endif

  return 1;
}
