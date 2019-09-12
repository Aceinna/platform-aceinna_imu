/** ***************************************************************************
 * @file SiRFPacketFormat.h SiRF GPS receiver.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * This file includes all specific processing, including configuring,
 *   for SiRF GPS receiver.
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#include "Indices.h"

#define SIRF_NAVIGATION_DATA          0x02
#define SIRF_MEASURED_TRACKER_DATA    0x04
#define SIRF_SOFTWRE_VERSION_STRING   0x06
#define SIRF_SOFTWARE_COMMAND_ACK     0x0B // ID 12
#define SIRF_SOFTWARE_COMMAND_NACK    0x0C // ID 13
#define SIRF_GEODETIC_NAVIGATION_DATA 0x29 // ID 41
#define SIRF_TRACKER_MESSAGES         0x41 // ID 65 'A' also ERROR
#define SIRF_SBAS_PARAMETERS          0x32
#define SIRF_EPHEMERIS                0x38 // ID 56
#define SIRF_STATISICS                0xE1 // ID 225
#define SIRF_STATIC_NAVIGATION        0x8f // ID 143

#define SIRF_POLL_SOFTWARE_VERSION  0x84 // ID 132
#define SIRF_SET_DGPS_SOURCE        0x85 // ID 133
#define SIRF_SET_DGPS_CTRL          0x8A // ID 138
#define SIRF_SET_MESSAGE_RATE       0xA6 // ID 166
#define SIRF_SET_SBAS_PARAMETERS    0xAA // ID 170
#define SIRF_SET_SERIAL_PORT        0x86 // ID 134

#define SIRF_SET_NAV_MODE           0x88 // ID 136

#define MAX_MESSAGE_RATE 30
#define NUM_CNO_VALUES   10

#define TEN_TO_THE_SEVENTH   10000000
#define ONE_OVER_TEN_TO_THE_SEVENTH   1e-7

#define MILLISECOND_TO_SECOND    1000
#define SECOND_TO_MILLISECOND    0.001

#pragma pack(1) ///< structure will appear in memory as they do here
typedef struct {
    int32_t	 pos[NUM_AXIS];
    int16_t	 vel[NUM_AXIS];
    uint8_t	 mode1;
    uint8_t	 hdop;
    uint8_t	 mode2;
    uint16_t gpsWeek;
    uint32_t tow;
    uint8_t	 prn[12];
} tSiRFNav;

#define MAX_NUM_TRACKER_CHANNELS 12
typedef struct {
    uint8_t id;
    uint8_t azimuth;
    uint8_t elevation;
    uint8_t state;
    uint8_t cno[NUM_CNO_VALUES];
} tSiRFTrackerData;

typedef struct {
    uint16_t	     gpsWeek;
    uint32_t	     tow;
    uint8_t          numChannels;
    tSiRFTrackerData channel[MAX_NUM_TRACKER_CHANNELS];
} tSiRFTracker;

typedef struct {
    uint8_t  id;
    uint8_t  mode;
    uint8_t  messageIdToBeSet;
    uint8_t  updateRate;
    uint32_t reserved;
} tSiRFSetMessageRate;

typedef struct {
    uint8_t   id;
    uint16_t  reserved;
    uint8_t   degMode;
    uint8_t   posCalcMode;
    uint8_t   deadRecogEnable;
    int16_t   altitudeInput;
    uint8_t   altHoldMode;
    uint8_t   altHoldSource;
    uint8_t   coastTimeout;
    uint8_t   degradedTimeout;
    uint8_t   deadRecogTimeout;
    uint8_t   measAndTrackSmoothing;
} tSiRFSetModeControl;


typedef struct {
    uint8_t  id;
    uint8_t  sbasPRN;
    uint8_t  sbasMode;
    uint8_t  sbasBits;
    uint16_t reserved;
} tSiRFSetSbasParams;


typedef struct {
    uint8_t   id;
    uint8_t   dgpsSource;
    uint32_t  reserved;
    uint8_t   reserved2;
} tSiRFSetDgpsSource;


typedef struct {
    uint8_t   id;
    uint8_t   dgpsSelection;
    uint8_t   timeout;
} tSiRFSetDgpsControl;


typedef struct {
    uint8_t id;
    uint8_t reserved;
} tSiRFPollVersion;

typedef struct {
    uint8_t id;   // 0x8f 143d
    uint8_t flag; // 1 enable, 0 disable
} tSiRFStaticNavigation;

typedef struct {
    uint8_t  id;
    uint32_t baud;
    uint8_t  databits;
    uint8_t  stopbits;
    uint8_t  parity;
    uint8_t  reserved;
} tSiRFPSetSerialPort;

enum eNavValidBitmask { // only useful info in bit 0
    NAV_COMPLETELY_VALID            = 0x0000, // best case
    SOLUTION_NOT_YET_OVERDETERMINED =   0x01,
};

enum eGpsPositionFixType {
    NO_NAV_FIX                = 0,
    ONE_SV_KV_SLN             = 1,
    TWO_SV_KV_SLN             = 2,
    THREE_SV_KV_SLN           = 3,
    FOUR_PLUS_SV_KV_SLN       = 4,
    TWO_D_LEAST_SQUARES_SLN   = 5,
    THREE_D_LEAST_SQUARES_SLN = 6,
    DR_SOLUTION               = 7
};

enum eAltitudeHoldStatus {
    NO_ALT_HOLD               = 0,
    ALT_HOLD_FROM_KF          = 1,
    ALT_HOLD_FROM_USER        = 2,
    ALWAYS_HOLD_ALT_FROM_USER = 3
};

#pragma pack(1)
typedef struct {
    // zero valid, non zero degraded
    uint16_t navValid; // 0x0201 enum eNavValidBitmask
    union {
       uint16_t all;
       struct {
           uint16_t fixType                         :3; // enum eGpsPositionFixType
           uint16_t tricklePowerInUse               :1;
           uint16_t altitudeHoldStatus              :2; // enum eAltitudeHoldStatus
           uint16_t dopLimitsExceeeded              :1;
           uint16_t dgpsCorrectionsApplied          :1;
           uint16_t sensorDrSolutionType            :1; // SiRF Drive only
           uint16_t navigationSolutionOverDetermined:1;
           uint16_t velocityDr2TimeoutExceeded      :1;
           uint16_t fixEditedByMiFunctions          :1;
           uint16_t invalidVelocity                 :1;
           uint16_t altitudeHoldDisabled            :1;
           uint16_t sensorDRErrorStatus             :2; // SiRF Drive only
       } bits;
    } navType; // 0x0403
    uint16_t extendedWeekNumber; // 0x0605 from 06.01.1980 (Jan 6)
    uint32_t tow; // 0x0a090807 [sec] * 10^3
    uint16_t utcYear; // 0x0c0b yyyy
    uint8_t  utcMonth; // 0x0d mm
    uint8_t  utcDay;  // 0x0e dd
    uint8_t  utcHour; // 0x0f hh
    uint8_t  utcMinute; // 0x10 mm
    uint16_t utcSecond; // 0x1211 ss
    uint32_t satelliteIdBitmap; // 0x16151413
    int32_t  latitude;           // 0x1a191817 [degrees] * 10^7, + = N
    int32_t  longitude;          // 0x1e1d1c1b [degrees] * 10^7, + = E
    int32_t  altitudeEllipsoid;  // 0x2221201f [m] * 10^2
    int32_t  altitudeMSL;        // 0x26252423 [m] * 10^2
    int8_t   mapDatum; // 0x27
    uint16_t speedOverGround;    // 0x2928 [m/s] *10^2
    uint16_t courseOverGround;   // 0x2b2a [degrees] clockwise from true north x 10^2
    int16_t  magneticVariation;  // 0x2d2c not implemented
    int16_t  climbRate;          // 0x2f2e [m/s] *10^2
    int16_t  headingRate;        // 0x3130 SiRFDrive only
    uint32_t estHorizPosErrror;  // 0x35343332 EHPE [meters] x 10^2
    uint32_t estVertPosErrror;   // 0x39383736 EVPE [meters] x 10^2
    uint32_t estTimeError;       // 0x3d3c3b3a ETE [s] 10^2 // SiRFDrive only
    uint16_t estHorizVelError;   // 0x3f3e EHVE [m/s] x 102 (SiRFDRive only)
    int32_t  clockBias;          // 0x43424140 [m] x 10^2
    uint32_t clockBiasError;     // 0x47464544 [m] x 10^2 (SiRFDRive only)
    int32_t  clockDrift;         // 0x4b4a4948 [m/s] x 10^2
    uint32_t clockDriftError;    // 0x4f4e4d4c [m/s] x 10^2 (SiRFDRive only)
    uint32_t distance;           // 0x53525150 [meters] Distance traveled since reset (SiRFDRive only)
    uint16_t distanceError;      // 0x5554 [meters] (SiRFDRive only)
    uint16_t headingError;       // 0x5756 [degrees] x 10^2 (SiRFDRive only)
    uint8_t  numSvsInFix;        // 0x58 see also satelliteIdBitmap
    uint8_t  hdop;               // 0x59 Horizontal Dilution of Precision x 5 (0.2 resolution)
    uint8_t  additionalModeInfo; // 0x5a see SiRF manual
} tSiRFGeoNav; // 90 bytes + 1 for additional mode Info
#pragma pack()

// reorderBytes defined in processSIRFGPS.cpp
#define SET_VALUE(val, m, idx) {(val) = reorderBytes(m, (idx), sizeof(val));}
