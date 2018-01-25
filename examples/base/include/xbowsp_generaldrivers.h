/** ***************************************************************************
 * @file   xbowsp_generaldrivers.h  Configuration and calibration data structure
 * @Author denglish
 * @date   February, 2011
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * per the latest design. Was drivers.h in 440 code
 ******************************************************************************/
#ifndef XBOWSP_DRIVERS_H
#define XBOWSP_DRIVERS_H

#include <stdint.h>
#include "xbowsp_version.h"

#define SIZEOF_WORD            2 // [bytes]

/// hard and soft iron resolution (2^16 / 2)
#define IRON_SCALE         32768

#define ACCEL_CAL_TABLE_OFFSET 0
#define ACCEL_CAL_TABLE_SIZE  54

#define RATE_CAL_TABLE_OFFSET (NUM_AXIS * ACCEL_CAL_TABLE_SIZE)
#define RATE_CAL_TABLE_SIZE   33

#define MAG_CAL_TABLE_OFFSET (RATE_CAL_TABLE_OFFSET + (NUM_AXIS * RATE_CAL_TABLE_SIZE))
#define MAG_CAL_TABLE_SIZE    33

#define TEMP_BIAS_OFFSET       0
#define TEMP_SCALE_OFFSET      6

#define TEMP_COUNT             0
#define BIAS_VALUE             1

#define SENSOR_COUNT           0
#define SCALE_VALUE            1

/// specifying how the user sets up the device
struct userBehavior_BITS  {        /// bits   description
   uint16_t freeIntegrate     : 1; /// 0
   uint16_t useMags           : 1; /// 1
   uint16_t useGPS            : 1; /// 2 - Not used yet
   uint16_t stationaryLockYaw : 1; /// 3 - Not used yet
   uint16_t restartOnOverRange: 1; /// 4
   uint16_t dynamicMotion     : 1; /// 5 - Not used
   uint16_t rsvd              :10; /// 6:15
};

union UserBehavior
{
   uint16_t                 all;
   struct userBehavior_BITS bit;
};

struct productConfiguration_BITS  { /// bits   description
    uint16_t hasMags     :1;        /// 0 - lsb
    uint16_t hasGps      :1;        /// 1
    uint16_t algorithm   :1;        /// 2 - 380 only uses one algorithm
    uint16_t extAiding   :1;        /// 3
    uint16_t architecture:4;        /// 4-7 - 380 only has one architecture
    uint16_t rsvd        :7;        /// 8-14
    uint16_t isADAHRS    :1;        /// 15 - msb
};

union ProductConfiguration {
    uint16_t                         all;
    struct productConfiguration_BITS bit;
};


struct orientation_BITS  {      /// bits   description
    uint16_t forwardAxisSign:1; /// 0 is pos, 1 is neg
    uint16_t forwardAxis    :2; /// 0=X, 1=Y, 2=Z axes forward, 3=do not use
    uint16_t rightAxisSign  :1; /// 0 is pos, 1 is neg
    uint16_t rightAxis      :2; ///	0=Y, 1=Z, 2=X axes forward, 3=do not use
    uint16_t downAxisSign   :1; /// 0 is pos, 1 is neg
    uint16_t downAxis       :2; /// 0=Z, 1=X, 2=Y axes forward, 3=do not use
    uint16_t rsvd           :7; /// 9:15
};

union Orientation {
    uint16_t                all;
    struct orientation_BITS bit;
};

/// EEPROM Data Structure: configuration data (NOTE: Refer to the DMU Serial Interface
///                        spec before changing this structure).  All variables must
///                        fit in the space allocated.
#pragma pack(1)
typedef struct {
    uint16_t           calibrationCRC;    /// CRC on the calibration structure  0x0000
    uint16_t           packetRateDivider; /// continuous packet rate divider    0x0001
    uint16_t           baudRateUser;      /// user port                         0x0002
    uint16_t           packetType;		  /// continuous packet type            0x0003

    uint16_t           analogFilterClocks[3];                               //  0x0004, 0x0005, 0x0006

    union Orientation  orientation; 	  /// user defined axis orientation     0x0007

    union UserBehavior userBehavior;       // uint16_t                          0x0008
    int16_t            hardIronBias[2];    ///< [-1,1) Gauss                    0x0009, 0x000a
    uint16_t           softIronScaleRatio; ///< [0,2), [0-200%)                 0x000b
    uint16_t           headingTrackOffset;                                  //  0x000c
    uint16_t           turnSwitchThreshold; // 0, 0.4, 10 driving, 1 flying [deg/sec]   0x000d
    int16_t            softIronAngle;                                       //  0x000e
    uint16_t           rsvd;                                                //  0x000f

    uint16_t           hardwareStatusEnable;                                //  0x0010
    uint16_t           comStatusEnable;                                     //  0x0011
    uint16_t           softwareStatusEnable;                                //  0x0012
    uint16_t           sensorStatusEnable;                                  //  0x0013
    int16_t            baudRateGPS; // enum in driverGPS.h                      0x0014
    int16_t            protocolGPS; // enum enumGPSProtocol in driverGPS.h      0x0015
    int16_t            baroCorrection;                                      //  0x0016
    int16_t            OffsetAnglesExtMag[2];                               //  0x0017, 0x0018
    int16_t            OffsetAnglesAlign[3];                                //  0x0019, 0x001a, 0x001b
    int16_t            hardIronBiasExt[2];                                  //  0x001c, 0x001d
    uint16_t           softIronScaleRatioExt;                               //  0x001e
    int16_t            softIronAngleExt;                                    //  0x001f
    union Orientation  orientationExt; // uint16_t 32(128)?                     0x0020
    // 33 16-bit spaces used

    // The MTLT uses twelve 16-bit fields
    int16_t            portOneUsage;                                       // 0x0021
    int16_t            portTwoUsage;                                       // 0x0022
    int16_t            portThreeUsage;                                     // 0x0023
    int16_t            portFourUsage;                                      // 0x0024
    int16_t            portOneBaudRate;                                    // 0x0025
    int16_t            portTwoBaudRate;                                    // 0x0026
    int16_t            portThreeBaudRate;                                  // 0x0027
    int16_t            portFourBaudRate;                                   // 0x0028
    
    int16_t	       rollUpperAlarmLimit;                                // 0x0029
    int16_t	       rollLowerAlarmLimit;                                // 0x002a
    int16_t	       pitchUpperAlarmLimit;                               // 0x002b
    int16_t	       pitchLowerAlarmLimit;                               // 0x002c
    uint16_t 	       rollHysteresis; 		//Hysteresis for Roll alarm 0x002d
    uint16_t	       pitchHysteresis; 	//Hysteresis for Pitch alarm 0x002e    
    uint16_t	       alarmSelector;		//Cone/Independent axis alarm selector 0x002f
    uint16_t	       coneAngleLimit;		//angle = acos( cos( phi ) * cos( theta ) ) * 180/pi 0x0030
    uint16_t	       coneAngleHysteresis; //Hysteresis for cone angle alarm  0x0031
    
    uint16_t           ecuAddress;                                             // 0x0032
    uint16_t           ecuBaudRate;                                            // 0x0033
    uint16_t           algResetSaveCfgPs;                                       // 0x0034
    uint16_t           HardSoftBitPs;                                           // 0x0035
    uint16_t           statusPrPs;                                              // 0x0036
    uint16_t           PtDfPs;                                                  // 0x0037
    uint16_t           OrienUserBehvPs;                                         // 0x0038
    uint16_t           AngConeAlarmPs;                                          // 0x0039 
    int16_t            CanBaudRateDetectEnable;                                 // 0x003a
    int16_t            CanTermResistorEnable;                                   // 0x003b
    int16_t            CanOdr;                                                  // 0x003c
    uint16_t           canPacketType;                                           // 0x003d
//    uint16_t           AccelPs;                                                 // 0x003a


    uint16_t           userPacketFields[2][46]; // 2 * 47 = 94 <-- number of 16-bit slots used by this array 
    uint16_t           tmp0; // 2 * 47 = 94 <-- number of 16-bit slots used by this array 
    uint16_t           linAccelSwitchLimit; // 2 * 47 = 94 <-- number of 16-bit slots used by this array 
    uint16_t           tmp2;  // Previously unused 'filter' variable
} ConfigurationStruct;
#pragma pack()

extern ConfigurationStruct gConfiguration;


#define N_MISALIGN         18
#define N_RESERVED_CAL      6

#define N_TABLES_A    	   15   ///< 6 + 6 + 3
#define N_ROWS_TABLE_A    360 	///< 162 + 99 + 99
#define N_COLS_TABLE_A      2

#pragma pack(1)
typedef struct {
    uint32_t serialNumber;
    char     versionString[N_VERSION_STR]; // [128]

    /// index of first element for each sensor (including 7th sensor)
    uint16_t calibrationTableIndexA[N_TABLES_A + 1]; // 15 + 1
    /// table order: (Temp Tables: xAccel, yAccel, zAccel, xRate, yRate, zRate),
    /// Inertial Tables: (same order), mag tables (x, y, z)
    uint32_t calibrationTablesA[N_ROWS_TABLE_A][N_COLS_TABLE_A]; // [360][2]
    int32_t  misalign[N_MISALIGN]; // [18]

    uint16_t reserved[N_RESERVED_CAL]; // [6]
    union    ProductConfiguration productConfiguration;

    /// added from the 440 - NAV-view may not program them
    float    AccelSensorRange;
    float    GyroSensorRange;
    float    MagSensorRange;

    uint16_t RollIncidenceLimit;
    uint16_t PitchIncidenceLimit;

    uint16_t HardIronLimit;
    uint16_t SoftIronLimit;
} CalibrationStruct;
#pragma pack()

extern CalibrationStruct gCalibration;

typedef struct {
    /// Input
    unsigned int  TableLength;	       ///< table size (starting with 0)
    uint8_t       firstColumnUnsigned; ///< TRUE: unsigned, FALSE, signed.
    unsigned int  bIndexLastSearch;    ///< index of the beginning row
                                       /// (where the last search ended)
    unsigned int  numberToBeSearched;  ///< search input--left column output
    unsigned int  bLeftValue;
    unsigned int  eLeftValue;  ///< search result: first row's columns(beginning and end)
    int           bRightValue;
    int           eRightValue; ///< search result:  second row's columns (beginning and end)
}LUTinOutStruct;


/**
 * 'VR' and 'VA' only use UINT8 field bootloader reads unspecified UINT (but
 *  assumed 2 cells wide) and sensor_init() assumes 16 bits
 */
typedef struct {
    unsigned int major;
    unsigned int minor;
    unsigned int patch;
    unsigned int stage;
    unsigned int build;
} softwareVersionStruct;

/// servicing/calling frequency of serial port transmit routine
#define SERIAL_TX_ROUTINE_FREQUENCY 100 ///< Hz

#endif

