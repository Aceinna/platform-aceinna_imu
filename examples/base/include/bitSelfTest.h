/** ***************************************************************************
 * @file bitSelfTest.h Built in and self test structures and constants.
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
#ifndef _BIT_SELF_TEST_H_
#define _BIT_SELF_TEST_H_

 #include <stdint.h>

  void  InitStructureVariables_BIT( void );

  /// Sensor Self-test functions
  void    BITSetRunFlags( void );
  void    BITInit( void );
  void    BITStartStop();
  void    BITCollectData( int16_t* reading );
  uint8_t getBITFlag();

typedef struct {
    uint8_t  PerformSelfTest;
    uint8_t  StartSelfTest;
    uint8_t  CompleteSelfTest;
    uint8_t  AnalyzeSelfTestResults;
    uint8_t  CollectSelfTestData; // should these be an enumeration?

    uint8_t  SelfTestDataCount;        ///< limit of SelfTest_PointsToCollect
    uint8_t  SelfTestDataIndex;        ///< 0 - biased, 1 - unbiased data
    int32_t  SelfTestSummation[2][11]; ///< [0][n] - biased, [1][n] - unbiased data
    uint16_t SelfTestDelta[11];        ///< actual calculated data
    uint16_t SelfTestDeltaLimit[11];   ///< test threshold vales for each axis
    uint8_t  NumberOfFailures;         ///< keep track of how many have occured
    uint8_t  SelfTestFailureLimit;     ///< acceptable number of tests failed

    /// @brief bit shift required to average the data collected during the
    ///   self-test. SelfTest_PointsToCollect is computed based on the value
    ///   assigned to SelfTest_AverageShift.
    uint8_t SelfTest_AverageShift;

    /// @brief number of data points to collect with and without the bias applied to the
    ///     rate-sensor and accelerometer:
    ///     SelfTest_PointsToCollect = 2 << SelfTest_AverageShift;
    uint8_t SelfTest_PointsToCollect;
} BuiltInTestStruct;

#define SPI_REG_SELF_TEST_MASK 0x0C   // b01000111

#endif
