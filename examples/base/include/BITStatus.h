/** ***************************************************************************
 * @file   BITStatus.h DMU380 header file for BIT and Status definitions.
 * @Author Darren Liccardo
 * @date   September, 2013
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Data structure for Built In Test (BIT)
 *************************************************************************** **/

#ifndef _BIT_STATUS_H_
#define _BIT_STATUS_H_

#include <stdint.h>

#define ANALOG_BIT_ERROR_COUNT_LIMIT 1000 // analog hardware soft bit limits
#define COM_BIT_ERROR_COUNT_LIMIT    1000 // com soft bit limits

struct BIT_STATUS_BITS  {        // bits description
    uint16_t masterFail     : 1; //   0  master fatal error
    uint16_t hardwareError  : 1; //   1  hardware error
    uint16_t comError       : 1; //   2  communication error with outside world
    uint16_t softwareError  : 1; //   3  software error
    uint16_t rsvd           : 4; // 4:7
    uint16_t masterStatus   : 1; //   8   master status
    uint16_t hardwareStatus : 1; //   9   hardware status
    uint16_t comStatus      : 1; //  10   communication status
    uint16_t softwareStatus : 1; //  11   software status
    uint16_t sensorStatus   : 1; //  12   sensor status
    uint16_t rsvd2          : 3; //  13:15
};

union BIT_STATUS {
    uint16_t               all;
    struct BIT_STATUS_BITS bit;
};

struct HW_BIT_BITS {                  // bits  description
    uint16_t powerError         : 1;  //   0   power error
    uint16_t environmentalError : 1;  //   1   environmental error
    uint16_t sensorError        : 1;  //   2   sensor error (T0 N/A)
    uint16_t MagnetometerError  : 1;  //   3   sensor error 
    uint16_t AccelerometerError : 1;  //   4   sensor error 
    uint16_t RateGyroError      : 1;  //   5   sensor error 
    uint16_t rsvd               : 10; // 6:15
};

union HW_BIT {
    uint16_t           all;
    struct HW_BIT_BITS bit;
};

#if 0
struct HW_PW_BIT_BITS {      // bits
    uint16_t inpPower   : 1; //   0  input power
    uint16_t inpCurrent : 1; //   1  input current
    uint16_t inpVoltage : 1; //   2  input voltage
    uint16_t fiveVolt   : 1; //   3  five volt bus
    uint16_t threeVolt  : 1; //   4  three volt bus
    uint16_t twoVolt    : 1; //   5  two volt bus
    uint16_t twoFiveRef : 1; //   6  2.5v reference
    uint16_t sixVolt    : 1; //   7  six volt bus
    uint16_t grdRef     : 1; //   8  ground reference
    uint16_t fourVolt   : 1; //   9  4Volt (T0 N/A)
    uint16_t rsvd       : 6; // 10:15
};

union HW_PW_BIT {
    uint16_t              all;
    struct HW_PW_BIT_BITS bit;
};
#endif

struct HW_ENV_BIT_BITS {   // bits
    uint16_t pcbTemp : 1;  //   0  PCB temperature
    uint16_t rsvd    : 15; // 1:15
};

union HW_ENV_BIT {
    uint16_t               all;
    struct HW_ENV_BIT_BITS bit;
};


struct COM_BIT_BITS {           // bits
    uint16_t serialAError : 1;  //   0  serial port A error
    uint16_t serialBError : 1;  //   1  serial port B error
    uint16_t i2CError     : 1;  //   2  I2C bus error
    uint16_t spiError     : 1;  //   3  SPI bus error
    uint16_t rsvd         : 12; // 4:15
};

union COM_BIT {
    uint16_t            all;
    struct COM_BIT_BITS bit;
};

struct SCI_BITS  {                  // bits   description
    uint16_t transBufOverflow : 1;  //   0    transmit buffer overflow
    uint16_t recBufOverflow   : 1;  //   1    receive buffer overflow
    uint16_t framingError     : 1;  //   2    rec. framing error
    uint16_t breakDetect      : 1;  //   3    rec. break detect
    uint16_t parityError      : 1;  //   4    rec. parity error
    uint16_t rsvd             : 11; // 5:15
};

union SCI_BIT {
    uint16_t        all;
    struct SCI_BITS bit;
};

struct SW_BIT_BITS {              // bits
    uint16_t algorithmError : 1;  // 0  algorithm error
    uint16_t dataError      : 1;  // 1  data error
    uint16_t rsvd           : 14; // 2:15
};

union SW_BIT {
    uint16_t           all;
    struct SW_BIT_BITS bit;
};

struct SW_ALG_BIT_BITS {                // bits
    uint16_t initialization       : 1;  //   0  algorithm initialization
    uint16_t overRange            : 1;  //   1  sensor overrange
    uint16_t missedNavigationStep : 1;  //   2  missed navigation step
    uint16_t rsvd                 : 13; // 3:15
};

union SW_ALG_BIT {
    uint16_t all;
    struct SW_ALG_BIT_BITS bit;
};

struct SW_DATA_BIT_BITS {             // bits
    uint16_t calibrationCRCError : 1; // 0     calibration structure CRC error
    uint16_t magAlignOutOfBounds : 1; // 1     mag align out of bounds
    uint16_t rsvd                : 14; // 2:15
};

union SW_DATA_BIT {
    uint16_t                all;
    struct SW_DATA_BIT_BITS bit;
};

struct HW_STATUS_BITS {	              // bits
    uint16_t unlocked1PPS        : 1; // 0  unlocked 1PPS signal
    uint16_t unlockedInternalGPS : 1; // 1  unlocked (no 3D fix) internal GPS receiver
    uint16_t noDGPS              : 1; // 2  no WAAS corrections aval. for interal GPS
    uint16_t unlockedEEPROM      : 1; // 3  unlocked EEPROM (enabled WE command)
    uint16_t invalidAirdata      : 1; // 4  invalid airdata
    uint16_t rsvd                : 11; // 5:15
};

union HW_STATUS {
    uint16_t              all;
    struct HW_STATUS_BITS bit;
};

struct COM_STATUS_BITS {                  //bits
    uint16_t noExternalGPS          : 1;  // 0  no ext. GPS messages being received
    uint16_t noExternalMagnetometer : 1;  // ext. Mag messages being received
    uint16_t rsvd                   : 14; // 2:15
};

union COM_STATUS {
    uint16_t               all;
    struct COM_STATUS_BITS bit;
};

struct SW_STATUS_BITS {                 // bits
    uint16_t algorithmInit         : 1; // 0  algorithm initialization
    uint16_t highGain              : 1; // 1  high gain mode
    uint16_t attitudeOnlyAlgorithm : 1; // 2  attitude only algorithm
    uint16_t turnSwitch            : 1; // 3  turn switch
    uint16_t noAirdataAiding       : 1; // 4  airdata aiding
    uint16_t noMagnetometerheading : 1; // 5  magnetometer heading
    uint16_t noGPSTrackReference   : 1; // 6  GPS track
    uint16_t rsvd                  : 9; // 7:15
};

union SW_STATUS {
    uint16_t              all;
    struct SW_STATUS_BITS bit;
};

struct SENSOR_STATUS_BITS { // bits
    uint16_t overRange : 1; // 0    sensor over-range
    uint16_t rsvd      : 15; // 1:15
};

union SENSOR_STATUS {
    uint16_t                  all;
    struct SENSOR_STATUS_BITS bit;
};

#if 0
struct HW_SENSOR_ERROR_BITS {        // bits
    uint16_t accel1Current       : 1; // 0
    uint16_t rate1Current        : 1; // 1
    uint16_t accel2Current       : 2; // 2
    uint16_t rate2Current        : 3; // 3
    uint16_t accel3Current       : 4; // 4
    uint16_t rate3Current        : 5; // 5
    uint16_t staticPressCurrent  : 6; // 6
    uint16_t dynamicPressCurrent : 7; // 7
    uint16_t rsvd                : 8; // 8-15
};

union HW_SENSOR_ERROR {
    uint16_t                    all;
    struct HW_SENSOR_ERROR_BITS bit;
};
#endif

//BIT and Status structure
typedef struct {
    union BIT_STATUS      BITStatus;         // master bit and status word
    union HW_BIT          hwBIT;
//  union HW_PW_BIT       hwPwrBIT;          // hardware power
    union HW_ENV_BIT      hwEnvBIT;          // hardware environmental
    union COM_BIT         comBIT;
    union SCI_BIT         comSABIT;          // com serial port A
    union SCI_BIT         comSBBIT;          // com serial port B
    union SW_BIT          swBIT;
    union SW_ALG_BIT      swAlgBIT;          // software algorithm
    union SW_DATA_BIT     swDataBIT;         // software data
    union HW_STATUS       hwStatus;          // hardware status
    union COM_STATUS      comStatus;         // communication status with outside world
    union SW_STATUS       swStatus;          // software status
    union SENSOR_STATUS   sensorStatus;      // sensor status
//  union HW_SENSOR_ERROR hwSensorErrStatus; // hardware sensor error status
} BITStatusStruct;

#endif

