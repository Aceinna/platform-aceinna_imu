/** ***************************************************************************
 * @file   configuration.h  Configuration and calibration data structure
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * per the latest design. Was drivers.h in 440 code
* 06/01/2020  |                                             | Daich
* Description: add configGetPacketCode function to get user packet type
*******************************************************************************/
#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

#include <stdint.h>

#define SIZEOF_WORD            2 // [bytes]

/// hard and soft iron resolution (2^16 / 2)
#define IRON_SCALE         32768


#define TEMP_COUNT             0
#define BIAS_VALUE             1

#define SENSOR_COUNT           0
#define SCALE_VALUE            1

#define UART_COMM 0
#define SPI_COMM  1
#define CAN_BUS   2

/// constants for port type configuration fields (see DMU specification)
#define PORT_BAUD_CONFIG
#define BAUD_UNDEFINED -1
#define BAUD_9600	0
#define BAUD_19200	1
#define BAUD_38400	2
#define BAUD_57600	3
#define BAUD_4800	4
#define BAUD_115200	5
#define BAUD_230400	6
#define BAUD_460800	7

#define NUM_BAUD_RATES      8

// xbowsp_fields.c (set default at startup to 20 Hz, AU pckt at 57.6 kbps)
#define DEFAULT_PACKET_RATE_DIVIDER     2
#define DEFAULT_PACKET_CODE             0x4631
#define DEFAULT_BAUD_RATE               BAUD_115200

// here is definition for packet rate divider
// considering that data acquisition task runs at 200 Hz 
typedef enum {
    PACKET_RATE_DIV_INVALID = -1,
    PACKET_RATE_DIV_QUIET   = 0,      // quiet mode
    PACKET_RATE_DIV_200HZ   = 200,    // packet rate 200 Hz
    PACKET_RATE_DIV_100HZ   = 1,      // packet rate 100 Hz
    PACKET_RATE_DIV_50HZ    = 2,      // packet rate 50 Hz
    PACKET_RATE_DIV_25HZ    = 4,      // packet rate 25 Hz
    PACKET_RATE_DIV_20HZ    = 5,     // packet rate 20 Hz
    PACKET_RATE_DIV_10HZ    = 10,     // packet rate 10 Hz
    PACKET_RATE_DIV_5HZ     = 20,     // packet rate 5  Hz
    PACKET_RATE_DIV_2HZ     = 50,    // packet rate 2  Hz
    PACKET_RATE_DIV_1HZ     = 100,    // packet rate 1  Hz
}packet_rate_div_t;

// supported baud rate
typedef enum {
  _ECU_500K      =    0,                    // 500kbps
  _ECU_250K      =    1,                    // 250kbps
  _ECU_125K      =    2,                    // 125kbps
  _ECU_1000K     =    3                     // 1000kbps
} _ECU_BAUD_RATE;

// MTLT's ODR on CAN
enum {
  CAN_PACKET_RATE_0           =           0,   //quiet
  CAN_PACKET_RATE_2           =           2,   // 2Hz
  CAN_PACKET_RATE_5           =           5,   // 5Hz
  CAN_PACKET_RATE_10          =           10,  // 10Hz
  CAN_PACKET_RATE_20          =           20,  // 20Hz
  CAN_PACKET_RATE_25          =           25,  // 25Hz
  CAN_PACKET_RATE_50          =           50,  // 50Hz
  CAN_PACKET_RATE_100         =           100, // 100Hz
  CAN_PACKET_RATE_200         =           200  // 200Hz
};


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


struct orientation_BITS  {      /// bits   description
    uint16_t forwardAxisSign:1; /// >> 0 0 is pos, 1 is neg
    uint16_t forwardAxis    :2; /// >> 1 0=X, 1=Y, 2=Z axes forward, 3=do not use
    uint16_t rightAxisSign  :1; /// >> 3 0 is pos, 1 is neg
    uint16_t rightAxis      :2; ///	>> 4 0=Y, 1=Z, 2=X axes forward, 3=do not use
    uint16_t downAxisSign   :1; /// >> 6 is pos, 1 is neg
    uint16_t downAxis       :2; /// >> 7 0=Z, 1=X, 2=Y axes forward, 3=do not use
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
    uint16_t           packetCode;		  /// continuous packet 2 bytes code    0x0003

    uint16_t           analogFilterClocks[3];                               //  0x0004, 0x0005, 0x0006

    union Orientation  orientation; 	  /// user defined axis orientation     0x0007

    union UserBehavior userBehavior;       // uint16_t                          0x0008
    int16_t            hardIronBias[2];    ///< [-1,1) Gauss                    0x0009, 0x000a
    uint16_t           softIronScaleRatio; ///< [0,2), [0-200%)                 0x000b
    uint16_t           headingTrackOffset;                                  //  0x000c
    uint16_t           turnSwitchThreshold; // 0, 0.4, 10 driving, 1 flying [deg/sec]   0x000d
    int16_t            softIronAngle;                                       //  0x000e
    int16_t            testFreq;                                            //  0x000f

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
    
    int16_t	           rollUpperAlarmLimit;                                // 0x0029
    int16_t	           rollLowerAlarmLimit;                                // 0x002a
    int16_t	           pitchUpperAlarmLimit;                               // 0x002b
    int16_t	           pitchLowerAlarmLimit;                               // 0x002c
    uint16_t 	       rollHysteresis; 		//Hysteresis for Roll alarm 0x002d
    uint16_t	       pitchHysteresis; 	//Hysteresis for Pitch alarm 0x002e    
    uint16_t	       alarmSelector;		//Cone/Independent axis alarm selector 0x002f
    uint16_t	       coneAngleLimit;		//angle = acos( cos( phi ) * cos( theta ) ) * 180/pi 0x0030
    uint16_t	       coneAngleHysteresis; //Hysteresis for cone angle alarm  0x0031
    
    uint16_t           ecuAddress;                                              // 0x0032
    uint16_t           ecuBaudRate;                                             // 0x0033
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

    uint16_t           spiDataReadyConfig;                                      // 0x003e
    uint16_t           spiDataRate;                                             // 0x003f
    uint16_t           spiRateDynamicRange;                                     // 0x0040   
    uint16_t           spiLpfType;                                              // 0x0041

    uint16_t           activeChips;                                             // 0x0042
    uint16_t           usedChips;                                               // 0x0043

    uint16_t           usedSensors[16];                                         // 0x44 - 0x53
    uint16_t           rsvd1[24];
    uint16_t           rsvd2[46];
    uint16_t           rsvd3; 
    uint16_t           rsvd4; 
    uint16_t           rsvd5; 
} ConfigurationStruct;
#pragma pack()

extern ConfigurationStruct gConfiguration;


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
int32_t baudEnumToBaudRate(int baudEnum);

#define FORWARD   0
#define RIGHT     1
#define DOWN      2

#define PLUS_X    0x582B
#define PLUS_Y    0x592B
#define PLUS_Z    0x5A2B
#define MINUS_X   0x582D
#define MINUS_Y   0x592D
#define MINUS_Z   0x5A2D


#define FWD_X_PLUS_MASK	   0x00000000
#define FWD_X_MINUS_MASK   0x00000001
#define FWD_Y_PLUS_MASK	   0x00000002
#define FWD_Y_MINUS_MASK   0x00000003
#define FWD_Z_PLUS_MASK	   0x00000004
#define FWD_Z_MINUS_MASK   0x00000005

#define RIGHT_X_PLUS_MASK  0x00000020
#define RIGHT_X_MINUS_MASK 0x00000028
#define RIGHT_Y_PLUS_MASK  0x00000000
#define RIGHT_Y_MINUS_MASK 0x00000008
#define RIGHT_Z_PLUS_MASK  0x00000010
#define RIGHT_Z_MINUS_MASK 0x00000018

#define DOWN_X_PLUS_MASK   0x00000080
#define DOWN_X_MINUS_MASK  0x000000C0
#define DOWN_Y_PLUS_MASK   0x00000100
#define DOWN_Y_MINUS_MASK  0x00000140
#define DOWN_Z_PLUS_MASK   0x00000000
#define DOWN_Z_MINUS_MASK  0x00000040


// placholders for Nav_view compatibility
extern softwareVersionStruct dupFMversion; 
extern softwareVersionStruct ioupFMversion;
extern softwareVersionStruct bootFMversion;
uint16_t configGetPacketCode();
#endif




