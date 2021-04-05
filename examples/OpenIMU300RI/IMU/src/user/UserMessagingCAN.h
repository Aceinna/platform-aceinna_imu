/*******************************************************************************
 * File:   UserConfiguration.h
 * Created on Jan 25, 2017
 ******************************************************************************/
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

#ifndef USER_MESSAGING_H
#define USER_MESSAGING_H

#include <stdint.h>

#include "GlobalConstants.h"


extern int userPacketOut;


typedef struct {
    uint16_t masterFail             :    1; // 0 = normal, 1 = fatal error has occurred
    uint16_t hardwareError          :    1; // 0 = normal, 1 = internal hardware error
    uint16_t reserved               :    1; 
    uint16_t softwareError          :    1; // 0 = normal, 1 = internal software error
    uint16_t inpPower               :    1; // 0 = normal, 1 = out of bounds
    uint16_t inpCurrent             :    1; // 0 = normal, 1 = out of bounds
    uint16_t inpVoltage             :    1; // 0 = normal, 1 = out of bounds
    uint16_t fiveVolt               :    1; // 0 = normal, 1 = out of bounds
    uint16_t threeVolt              :    1; // 0 = normal, 1 = out of bounds
    uint16_t twoVolt                :    1; // 0 = normal, 1 = out of bounds
    uint16_t twoFiveref             :    1; // 0 = normal, 1 = out of bounds
    uint16_t sixVolt                :    1; // 0 = normal, 1 = out of bounds
    uint16_t grdRef                 :    1; // 0 = normal, 1 = out of bounds
    uint16_t pcbTemp                :    1; // 0 = normal, 1 = out of bounds
    uint16_t reserved1              :    2;
} HARDWARE_TEST_PAYLOAD;

typedef struct {
    uint16_t softwareError             :     1; // 0 = normal, 1 = internal software error
    uint16_t algorithmerror            :     1; // 0 = normal, 1 = error
    uint16_t dataError                 :     1; // 0 = normal, 1 = error
    uint16_t initialization            :     1; // 0 = normal, 1 = error during algorithm initialization
    uint16_t overRange                 :     1; // 0 = normal, 1 = fatal sensor over range
    uint16_t missedNavigationStep      :     1; // 0 = normal, 1 = deadline missed for navigation
    uint16_t calibrationCRCError       :     1; // 0 = normal, 1 = incorrect CRC on calibration EEPROM data
    uint16_t reserved                  :     9;
} SOFTWARE_TEST_PAYLOAD;

typedef struct {
    uint16_t masterStatus             :     1; // 0 = normal, 1 = hardware, sensor, CAN, or software alert
    uint16_t hardwareStatus           :     1; // 0 = normal, 1 = programmable alert
    uint16_t softwareStatus           :     1; // 0 = normal, 1 = programmable alert
    uint16_t sensorStatus             :     1; // 0 = normal, 1 = programmable alert
    uint16_t unlocked1PPS             :     1; // 0 = not asserted, 1 = asserted
    uint16_t unlockedInternalGPS      :     1; // 0 = not asserted, 1 = asserted
    uint16_t noDGPS                   :     1; // 0 = DGPS lock, 1 = no DGPS
    uint16_t unlockedEEPROM           :     1; // 0 = locked, 1 = unlocked
    uint16_t algorithmInit            :     1; // 0 = normal, 1 = in initialization mode
    uint16_t highGain                 :     1; // 0 = low gain mode, 1 = high gain mode
    uint16_t attitudeOnlyAlgorithm    :     1; // 0 = navigation state tracking, 1 = attitude only state tracking
    uint16_t turnSwitch               :     1; // 0 = off, 1 = yaw rate greater than turnSwitch threshold
    uint16_t sensorOverRange          :     1; // 0 = not asserted, 1 = asserted
    uint16_t reserved                 :     3;
} STATUS_TEST_PAYLOAD; 


// ODR set packet type
typedef struct {
    uint8_t dest_address;              // target's address
    uint8_t odr;                       // odr values
} RATE_CONFIG_PAYLOAD;

// data packet payload
typedef  struct {
    uint8_t dest_address;             // targer address
    union {
        struct {
            uint8_t slope_sensor       :   1;     // ss2 packet
            uint8_t angular_rate       :   1;     // angular rate packet
            uint8_t accelerator        :   1;     // acceleration packet
            uint8_t reserved           :   5;
        } b;
        
        uint8_t r;
    } type_bits;
} PACKET_TYPE_PAYLOAD;


// LPF set payload format
typedef struct {
    uint8_t dest_address;                  // target's address
    uint8_t rate_cutoff;                   // LPF of rate sensor
    uint8_t accel_cutoff;                  // LPF of XL sensor
} DIGITAL_FILTER_PAYLOAD;


// orientation set payload format
typedef struct {
    uint8_t dest_address;                  // target's address
    uint8_t orien_bits[2];                 // orientation setting 
} ORIENTATION_SETTING;


// user behavior set payload format
typedef struct {
    uint8_t  dest_address;                  // target's address
    uint8_t  data[7]; 
} USER_BEHAVIOR_PAYLOAD;


// mag align payload
typedef struct {
    uint8_t dest_address;                  // target's address
    uint8_t cmd[7];                        // command 
} MAG_ALIGN_PAYLOAD;


// Angle alarm set payload format
typedef struct {
    uint8_t dest_address;                 // targert's address
    uint8_t roll_upper;                   // upper limitation of roll
    uint8_t roll_lower;                   // lower limitation of roll
    uint8_t pitch_upper;                  // upper limitation of pitch
    uint8_t pitch_lower;                  // lower limitation of pitch
    uint8_t roll_hysteresis;              // hysteresis of roll
    uint8_t pitch_hyseresis;              // hysteresis of pitch
} ANGLE_ALARM_PAYLOAD;


// Cone alarm set payload format
typedef struct {
    uint8_t dest_address;                  // target's address
    uint16_t alarm_selector;               // alarm selector
    uint16_t angle_limit;                  // angular limitation
    uint16_t angle_hysteresis;             // angular hysteresis
} CONE_ALARM_PAYLOAD;


// acceleration set payload format
typedef struct {
    uint8_t dest_address;                  // target's address
    uint16_t x_acceleration;               // x-axis acceleration
    uint16_t y_acceleration;               // y-axis acceleration
    uint16_t z_acceleration;               // z-axis acceleration
} ACCELERATION_PARAM_PAYLOAD;

extern void  ProcessRequest(void *dsc);
extern void  ProcessEcuCommands(void * command, uint8_t ps, uint8_t addr);
extern void  PrepareJ1939DataPackets(void);
extern void  EnqeuePeriodicDataPackets(int latency, int sendPeriodicPackets);



#endif /* USER_CONFIGURATION_H */
