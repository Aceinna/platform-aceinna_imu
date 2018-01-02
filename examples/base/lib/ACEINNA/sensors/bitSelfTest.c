/** ***************************************************************************
 * @file   BIT.c Built In Test
 * @Author
 * @date   September, 2013
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * init data structure, set biases, collect sensor data, remove biases
 * analyze the relults and report vai SPI or to debug console
 * Sensor self-test (do this in the wait period so the data capture
 *   is not affected):
 *   1) Apply bias to sensors and collect a period of data
 *   2) Remove bias and collect data
 *   3) Compare data sets
 *      a) if correct bias is present then the test passes
 *      b) indicate results to user (by setting the appropriate
 *         register) and disable test
 ******************************************************************************/
#include <salvo.h>

#include "bitSelfTest.h"
#include "boardDefinition.h"
#include "spi.h"
#include "timer.h"
#include "UserCommunication_SPI.h"
#include "debug.h"
#include "s_eeprom.h"

// For the self-test functions
#include "Indices.h"
#include "gyroscope.h"
#include "accelerometer.h"
#include "magnetometer.h"

#include "stm32f2xx.h"
#include "stm32f2xx_spi.h"
#include "xbowsp_init.h"
#include "scaling.h" //  RADS_TO_DEGREES
#include <math.h> // pow()
#include "bsp.h" // LED4 - FIXME (remove when debugging complete)

volatile BuiltInTestStruct gBIT;

void _BITConfigureHW( void );
void _BITRestoreHW( void );
void _BITAnalyzeResults( void );
uint8_t getBITFlag() { return gBIT.PerformSelfTest;};


uint8_t tempUInt8;
uint8_t pointsToCollect;
/** ***************************************************************************
 * @name BITInit() - set up the self test configuration and test limits
 * @brief Called at the start of taskDataAcquisition(). Clear
 *        SelfTestSummation[2][11]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void BITInit( void )
{
    for( uint8_t rowNumber = 0; rowNumber < 2; rowNumber++ ) {
        for( uint8_t colNumber = 0; colNumber < 11; colNumber++ ) {
            gBIT.SelfTestSummation[rowNumber][colNumber] = 0;
        }
    }

    /// @brief
    /// The passing limits on delta were obtained via a combination of empirical
    ///   data and values obtained in the data sheets. Limits were selected to be
    ///   approximately 1/3 of the minimum change seen during testing.
    gBIT.SelfTestDeltaLimit[XACCEL] = 180;
    gBIT.SelfTestDeltaLimit[YACCEL] = 250;
    gBIT.SelfTestDeltaLimit[ZACCEL] = 1680;

    gBIT.SelfTestDeltaLimit[XRATE] = 2000;
    gBIT.SelfTestDeltaLimit[YRATE] = 2000;
    gBIT.SelfTestDeltaLimit[ZRATE] = 2000;

    /// Initialize the self-test variables
    gBIT.SelfTest_AverageShift    = 1; //0;
    gBIT.SelfTest_PointsToCollect = 2 << gBIT.SelfTest_AverageShift;

    gBIT.NumberOfFailures    = 0;
    /// self-tests performed before reporting an error (based on empirical data)
    gBIT.SelfTestFailureLimit = 8;
}

/** ***************************************************************************
 * @name BITSetRunFlags() - set the flags to run the sensor self tests
 * @brief
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void BITSetRunFlags()
{
    gBIT.PerformSelfTest        = 1;
    gBIT.StartSelfTest          = 1;
    gBIT.CompleteSelfTest       = 0;
    gBIT.CollectSelfTestData    = 0;
    gBIT.AnalyzeSelfTestResults = 0;
}

/** ***************************************************************************
 * @name BITStartStop() - state machine to start, stop or analyze the data
 * @brief
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void BITStartStop()
{
    static uint16_t autoBitSelfTest_cntr = 0;

    // Automatically perform the self-test early in the operation
    if( autoBitSelfTest_cntr < 200 ) {
        autoBitSelfTest_cntr++;
        if( autoBitSelfTest_cntr == 200 ) {
            //BITSetRunFlags();
        }
    }

    // 
    if( gBIT.PerformSelfTest ) {
        _BITConfigureHW(); // if start flag set
        _BITRestoreHW();   // if complete flag set

        if ( gCalibration.productConfiguration.bit.hasMags &&  gBIT.AnalyzeSelfTestResults ) {
            MagnetometerSelfTest(); // only run if exist and use analyze flag to run once
        }
        _BITAnalyzeResults();  // after data collection complete
    }
}

/** ***************************************************************************
 * @name BITCollectData() - collect and process the self test sensor data
 * @brief
 * @param [in] reading - raw data from the sensors
 * @retval N/A
 ******************************************************************************/
void BITCollectData( int16_t* reading )
{
    if( gBIT.CollectSelfTestData && gBIT.PerformSelfTest )
    {
        // Temporary variable to prevent the compiler from issuing an 'order of
        //   volatile access' WARNING.
        tempUInt8       = gBIT.SelfTestDataIndex;
        pointsToCollect = gBIT.SelfTest_PointsToCollect;

        /// @brief
        /// Collect data (only collect N data points - the first N points are
        ///   ignored to mitigate transients)
        if( gBIT.SelfTestDataCount >= pointsToCollect ) // collect after 2
        {
            if( gBIT.SelfTestDataCount < ( pointsToCollect << 1 ) ) { // collct to 4
                /// Keep a running total of N data points
                gBIT.SelfTestSummation[tempUInt8][XACCEL] += reading[XACCEL];
                gBIT.SelfTestSummation[tempUInt8][YACCEL] += reading[YACCEL];
                gBIT.SelfTestSummation[tempUInt8][ZACCEL] += reading[ZACCEL];

                gBIT.SelfTestSummation[tempUInt8][XRATE] += reading[XRATE];
                gBIT.SelfTestSummation[tempUInt8][YRATE] += reading[YRATE];
                gBIT.SelfTestSummation[tempUInt8][ZRATE] += reading[ZRATE];

                gBIT.SelfTestSummation[tempUInt8][XMAG] += reading[XMAG];
                gBIT.SelfTestSummation[tempUInt8][YMAG] += reading[YMAG];
                gBIT.SelfTestSummation[tempUInt8][ZMAG] += reading[ZMAG];
            } else {
                gBIT.CollectSelfTestData = 0;
                /// @brief
                /// Average the data.  Call 'CompleteSelfTest' to remove bias from
                ///   sensors or AnalyzeSelfTest to compare the results and
                ///   restart the test or return test passed/failed to the SPI
                ///   register.
                /// Average N data points (rate-sensor); the value is incremented by one
                uint8_t bitsToShift = gBIT.SelfTest_AverageShift + 1; // << 1

                gBIT.SelfTestSummation[tempUInt8][XACCEL] >>= bitsToShift;
                gBIT.SelfTestSummation[tempUInt8][YACCEL] >>= bitsToShift;
                gBIT.SelfTestSummation[tempUInt8][ZACCEL] >>= bitsToShift;

                gBIT.SelfTestSummation[tempUInt8][XRATE] >>= bitsToShift;
                gBIT.SelfTestSummation[tempUInt8][YRATE] >>= bitsToShift;
                gBIT.SelfTestSummation[tempUInt8][ZRATE] >>= bitsToShift;

                gBIT.SelfTestSummation[tempUInt8][XMAG] >>= bitsToShift;
                gBIT.SelfTestSummation[tempUInt8][YMAG] >>= bitsToShift;
                gBIT.SelfTestSummation[tempUInt8][ZMAG] >>= bitsToShift;

                if( gBIT.SelfTestDataIndex == 0 ) {
                    /// @brief
                    /// After collection of the biased sensor data is complete,
                    ///   set the flag to remove the sensor bias and switch the
                    ///   array index so the biased data isn't overwritten.
                    gBIT.CompleteSelfTest  = 1;
                    gBIT.SelfTestDataIndex = 1;
                } else {
                    /// @brief
                    /// Collection of non-biased data is complete, set the flag to
                    ///   analyze the data and reset the self-test data-array index.
                    gBIT.AnalyzeSelfTestResults = 1;
                    gBIT.SelfTestDataIndex      = 0;
                }
            }
        }
        gBIT.SelfTestDataCount++;
    }
}

/// @brief
/// to reset the rate-sensor data-ready bit (EF_DATA_GYRO_READY) in
///   EFLAGS_DATA_READY after self-test calls. This is necessitated by the way
///   self-test applies/removes the bias from the sensor (via spi_transfer),
///   which completes by setting the gyro-ready bit in EFLAGS_DATA_READY.
#include "salvodefs.h"
/** ***************************************************************************
 * @name BITConfigureHW() - initiate the sensor self test
 * @brief apply the biases to the accelerometers and rate sensors
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _BITConfigureHW( void )
{
    if( gBIT.StartSelfTest ) ///< if selected
    {   /// stop accelerometer data-read from interfering with the self-test
        AccelDataReadyInt(DISABLE);

        if( !getAccelI2CBusy() )
        {
            AccelSelfTest_Bias( APPLY );
            GyroSelfTest_Bias( APPLY );

            gBIT.StartSelfTest = 0; ///< Clear function-call flag

            /// @brief Reset self-test bits in the diagnostic-status registers
            /// (0x3C and 0x3D).  Keep other bits (checksum, sensor overrange,
            ///  etc) the same.
            gUserSpi.DataRegister[SPI_REG_SYSTEM_STATUS_MSB] = gUserSpi.DataRegister[SPI_REG_SYSTEM_STATUS_MSB] & 0x03;
            gUserSpi.DataRegister[SPI_REG_SYSTEM_STATUS_LSB] = gUserSpi.DataRegister[SPI_REG_SYSTEM_STATUS_LSB] & 0xDF;

            gBIT.SelfTestDataCount = 0;
            /// index 0: for biased data; 1: for normal data
            gBIT.SelfTestDataIndex = 0;

            /// Initialize the summation/averaged-result data
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][XACCEL] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][YACCEL] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][ZACCEL] = 0;

            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][XRATE] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][YRATE] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][ZRATE] = 0;

            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][XMAG] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][YMAG] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][ZMAG] = 0;

            gBIT.CollectSelfTestData = 1; ///< collect self-test data

            /// @brief
            /// Reset the rate-sensor data-ready bit (EF_DATA_GYRO_READY) in
            /// EFLAGS_DATA_READY. self-test applies/removes the bias from the
            /// sensor (via spi_transfer), which completes setting the
            /// gyro-ready bit in EFLAGS_DATA_READY.
            OSClrEFlag( EFLAGS_DATA_READY, EF_DATA_GYRO_READY );
    }
        AccelDataReadyInt(ENABLE);
    }
}

/** ***************************************************************************
 * @name BITRestoreHW() - clean up after the sensor self test
 * @brief remove the biases from the accerometers and rate sensors
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _BITRestoreHW( void )
{
    if( gBIT.CompleteSelfTest ) ///< Stop the self-test when selected
    {
        /// @brief To prevent the accelerometer data-read from interfering with
        /// the self-test command
        AccelDataReadyInt(DISABLE);

        if( !getAccelI2CBusy() )
        {
            AccelSelfTest_Bias( REMOVE );
            GyroSelfTest_Bias( REMOVE );

            gBIT.CompleteSelfTest = 0;  ///< Clear function-call flag

            /// @brief Reset the count and switch the index so the biased data
            /// isn't overwritten
            gBIT.SelfTestDataCount = 0;
            gBIT.SelfTestDataIndex = 1;

            /// Initialize the data arrays
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][XACCEL] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][YACCEL] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][ZACCEL] = 0;

            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][XRATE] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][YRATE] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][ZRATE] = 0;

            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][XMAG] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][YMAG] = 0;
            gBIT.SelfTestSummation[gBIT.SelfTestDataIndex][ZMAG] = 0;

            gBIT.CollectSelfTestData = 1; ///< collect self-test data

            /// @brief Reset data-ready bit (EF_DATA_GYRO_READY) in
            ///  EFLAGS_DATA_READY. self-test applies/removes the bias from the
            ///  sensor (via spi_transfer), which completes by setting the
            ///  gyro-ready bit in EFLAGS_DATA_READY.
            OSClrEFlag( EFLAGS_DATA_READY, EF_DATA_GYRO_READY );
        }
        AccelDataReadyInt(ENABLE);
    }
}

/** ***************************************************************************
 * @name BITAnalyzeSelfTestResults() - analyze the results of the sensor self test
 * @brief Compare the results before and after the bias is applied to the
 *        sensors. Report any errors in the status register.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _BITAnalyzeResults( void )
{
    uint8_t  sensorErrorCount = 0;
    int32_t  tempInt32        = 0;
    uint16_t passFailLimit    = 0;

    if( gBIT.AnalyzeSelfTestResults )
    {   /// Set the appropriate status bits
        gBIT.AnalyzeSelfTestResults = 0; ///< Clear function-call flag

        /// ========== Rate-Sensors ==========
        /// X-Rate
        tempInt32 = gBIT.SelfTestSummation[0][XRATE];

        gBIT.SelfTestDelta[XRATE] = (uint16_t)( abs( tempInt32 - gBIT.SelfTestSummation[1][XRATE] ) );
        // 14000 counts +- 50% ~7000 - 21000, self test limit is 2000
        passFailLimit = gBIT.SelfTestDeltaLimit[XRATE];
        if( gBIT.SelfTestDelta[XRATE] < passFailLimit ) {
            /// Error: Sensor didn't react to applied stimulus
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] | 0x04;
            sensorErrorCount++;
            gAlgorithm.bitStatus.hwBIT.bit.sensorError   = 1;
            gAlgorithm.bitStatus.hwBIT.bit.RateGyroError = 1;
        } else {
            /// No error: Sensor responded with a bias
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] & 0xFB;
        }

        /// Y-Rate
        tempInt32 = gBIT.SelfTestSummation[0][YRATE];
        gBIT.SelfTestDelta[YRATE] = (uint16_t)( abs( tempInt32 - gBIT.SelfTestSummation[1][YRATE] ) );
        passFailLimit = gBIT.SelfTestDeltaLimit[YRATE];
        if( gBIT.SelfTestDelta[YRATE] < passFailLimit ) {
            /// Error: Sensor didn't react
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] | 0x08;
            sensorErrorCount++;
            gAlgorithm.bitStatus.hwBIT.bit.sensorError   = 1;
            gAlgorithm.bitStatus.hwBIT.bit.RateGyroError = 1;
        } else {
            /// No error
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] & 0xF7;
        }

        /// Z-Rate
        tempInt32 = gBIT.SelfTestSummation[0][ZRATE];
        gBIT.SelfTestDelta[ZRATE] = (uint16_t)( abs( tempInt32 - gBIT.SelfTestSummation[1][ZRATE] ) );
        passFailLimit = gBIT.SelfTestDeltaLimit[ZRATE];
        if( gBIT.SelfTestDelta[ZRATE] < passFailLimit ) {
            /// Error: Sensor didn't react
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] | 0x10;
            sensorErrorCount++;
            gAlgorithm.bitStatus.hwBIT.bit.sensorError   = 1;
            gAlgorithm.bitStatus.hwBIT.bit.RateGyroError = 1;
        } else {
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] & 0xEF;
        }

        /// ========== Accelerometers ==========
        /// X-Accel
        tempInt32 = gBIT.SelfTestSummation[0][XACCEL];
        gBIT.SelfTestDelta[XACCEL] = (uint16_t)( abs( tempInt32 - gBIT.SelfTestSummation[1][XACCEL] ) );
        passFailLimit = gBIT.SelfTestDeltaLimit[XACCEL];
        if( gBIT.SelfTestDelta[XACCEL] < passFailLimit ) {
            /// Test failed, set data bit in the register
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] | 0x20;
            sensorErrorCount++;
            gAlgorithm.bitStatus.hwBIT.bit.sensorError        = 1;
            gAlgorithm.bitStatus.hwBIT.bit.AccelerometerError = 1;
        } else {
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] & 0xDF;
        }

        /// Y-Accel
        tempInt32 = gBIT.SelfTestSummation[0][YACCEL];
        gBIT.SelfTestDelta[YACCEL] = (uint16_t)( abs( tempInt32 - gBIT.SelfTestSummation[1][YACCEL] ) );
        passFailLimit = gBIT.SelfTestDeltaLimit[YACCEL];
        if( gBIT.SelfTestDelta[YACCEL] < passFailLimit ) {
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] | 0x40;
            sensorErrorCount++;
            gAlgorithm.bitStatus.hwBIT.bit.sensorError        = 1;
            gAlgorithm.bitStatus.hwBIT.bit.AccelerometerError = 1;
        } else {
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] & 0xBF;
        }

        /// Z-Accel
        tempInt32 = gBIT.SelfTestSummation[0][ZACCEL];
        gBIT.SelfTestDelta[ZACCEL] = (uint16_t)( abs( tempInt32 - gBIT.SelfTestSummation[1][ZACCEL] ) );
        passFailLimit = gBIT.SelfTestDeltaLimit[ZACCEL];
        if( gBIT.SelfTestDelta[ZACCEL] < passFailLimit ) {
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] | 0x80;
            sensorErrorCount++;
            gAlgorithm.bitStatus.hwBIT.bit.sensorError        = 1;
            gAlgorithm.bitStatus.hwBIT.bit.AccelerometerError = 1;

        } else {
            gUserSpi.DataRegister[0x3C] = gUserSpi.DataRegister[0x3C] & 0x7F;
        }

        /// Set the diagnostic error bit - compare against 0xFC = b11111100
        if( gUserSpi.DataRegister[0x3C] & 0xFC ) {
            gUserSpi.DataRegister[0x3D] = gUserSpi.DataRegister[0x3D] | 0x20;
            gBIT.NumberOfFailures = gBIT.NumberOfFailures + 1;
        } else {
            gUserSpi.DataRegister[0x3D] = gUserSpi.DataRegister[0x3D] & 0xDF;
            gBIT.NumberOfFailures = 0;
        }
        gBIT.PerformSelfTest = 0; // Disable

        /// If self-test fails, redo the test if below the failure limit
        tempUInt8 = gBIT.NumberOfFailures;
        if( tempUInt8 > 0 ) {
            if( tempUInt8 < gBIT.SelfTestFailureLimit ) {
                gBIT.PerformSelfTest = 1;
                gBIT.StartSelfTest   = 1;
            } else {
                /// @brief Too many self-test failures, leave the self-test
                /// error bit set and do not restart the test
                gBIT.PerformSelfTest = 0;
                gBIT.StartSelfTest   = 0;
            }
        } else {
            /// completed successfully; reset the self-test bit
DEBUG_INT("Failures= \t", gBIT.NumberOfFailures);
DEBUG_ENDLINE();

            gUserSpi.DataRegister[ SPI_REG_SELF_TEST_READ ] = 0x00;
        }
    }
}
