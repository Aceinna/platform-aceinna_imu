/** ***************************************************************************
 * @file   UARTMessages.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
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
#ifndef _UART_MESSAGES_H
#define _UART_MESSAGES_H
#include <stdint.h>

#pragma pack(1)

// payload structure of standard IMU data message
typedef struct {
    uint32_t timer;
    float    accel_mpss[3];
    float    rate_dps[3];
    float    mag_G[3];
}data1_payload_t;

// payload structure of standard test message
typedef struct {
    uint32_t counter;
}test_payload_t;

// payload structure of standard unit attitude message
typedef struct {
    uint32_t itow;
    double   dblItow;
    float    roll;
    float    pitch;
    float    corrRates[3];
    float    accels[3];
    uint8_t  ekfOpMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
}angle1_payload_t;

// payload structure of alternative IMU data message
typedef struct {
    uint32_t tstmp;
    double   dbTstmp;
    float    accel_g[3];
    float    rate_dps[3];
    float    mag_G[3];
    float    temp_C;
}scaled1_payload_t;

// payload structure of standard EKF message
typedef struct {
    uint32_t tstmp;
    double   dbTstmp;
    float    roll;
    float    pitch;
    float    yaw;
    float    accels_g[3];
    float    rates_dps[3];
    float    rateBias[3];
    float    mags[3];
    uint8_t  opMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
}ekf1_payload_t;

// payload structure of enchanced EKF message
typedef struct {
    uint32_t tstmp;
    double   dbTstmp;
    float    roll;
    float    pitch;
    float    yaw;
    float    accels_g[3];
    float    accelBias[3];
    float    rates_dps[3];
    float    rateBias[3];
    float    velocity[3];
    float    mags[3];
    double   pos[3];
    uint8_t  opMode;
    uint8_t  accelLinSwitch;
    uint8_t  turnSwitch;
}ekf2_payload_t;

// payload structure of standard unit attitude message
typedef struct {
    uint32_t itow;
    double   dblItow;
    float    roll;
    float    pitch;
    float    yaw;
    float    corrRates[3];
    float    accels[3];
}angle2_payload_t;


#pragma pack()

BOOL Fill_PingPacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_VersionPacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_zTPacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_z1PacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_a1PacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_a2PacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_s1PacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_e1PacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_e2PacketPayload(uint8_t *payload, uint8_t *payloadLen);

#endif
