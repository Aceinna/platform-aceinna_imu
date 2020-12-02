/** ***************************************************************************
 * @file   user_message.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 ******************************************************************************/
/*******************************************************************************
Copyright 2020 ACEINNA, INC

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

#ifndef _USER_MESSAGE_H
#define _USER_MESSAGE_H

#include <stdint.h>

#include "constants.h"
#include "ucb_packet.h"

#include "gnss_data_api.h"
#include "ins_interface_API.h"

// total size of user packet structure should not exceed 255 bytes
#pragma pack(1)

// example of user payload structure
typedef struct {
    uint32_t   paramNum;                                             // parameter number in parameters structure   (little endian)
    uint8_t    parameter[64];                                        // up to 30 64-bit parameters  (little endian)
} userParamPayload;

#pragma pack()

typedef enum {
    USR_IN_NONE = 0,
    USR_IN_PING,
    USR_IN_GET_VERSION,
    USR_IN_UPDATE_PARAM,
    USR_IN_SAVE_CONFIG,
    USR_IN_GET_ALL,
    USR_IN_RESTORE_DEFAULTS,
    USR_IN_GET_BLOCK,
    USR_IN_UPDATE_BLOCK,
    USR_IN_CAR_SPEED,
    USR_IN_MAX,
} UserInPacketType;

// User output packet codes, change at will
typedef enum {
    USR_OUT_NONE  = 0,
    USR_OUT_RAWIMU,
    USR_OUT_BESTGNSS,
    USR_OUT_INSPVAX,
    USR_OUT_ODO,
    USR_OUT_SATELLITES,
    USR_OUT_MAX
} UserOutPacketType;

//  user out packet struct
#pragma pack(1)

// payload structure of alternative IMU data message
typedef struct {
	uint16_t    gps_week;           // GPS Week number
    uint32_t    gps_millisecs;      // Milliseconds into week
    float       x_acceleration;		// change in velocity along x axis in scaled m/s
    float       y_acceleration;     // change in velocity along y axis in scaled m/s
    float       z_acceleration;		// change in velocity along z axis in scaled m/s
    float       x_gyro_rate;        // change in angle around x axis in deg
    float       y_gyro_rate;        // change in angle around y axis in deg
    float       z_gyro_rate;        // change in angle around z axis in deg
} output_rawimu_struct;

typedef struct {
    uint16_t    gps_week;                           // GPS Week number
    uint32_t    gps_millisecs;                      // Milliseconds into week
    uint8_t     position_type;                      // Position type
    double      latitude;                           // latitude (deg)
    double      longitude;                          // longitude (deg)
    double      height;                             // height above mean sea level (m)
    float       latitude_standard_deviation;        // latitude standard deviation (m)
    float       longitude_standard_deviation;       // longitude standard deviation (m)
    float       height_standard_deviation;          // height standard deviation (m)
    uint8_t     number_of_satellites;               // number of satellites tracked
    uint8_t     number_of_satellites_in_solution;   // number of satellites used in solution
    float       hdop;
    float       diffage;
    float       north_vel;
    float       east_vel;
    float       up_vel;
    float       north_vel_standard_deviation;
    float       east_vel_standard_deviation;
    float       up_vel_standard_deviation;
} output_bestgnss_struct;

typedef struct {
    uint16_t    gps_week;        	    // GPS Week number
    uint32_t    gps_millisecs;   	    // Milliseconds into week
	uint8_t     ins_status;
    uint8_t     ins_position_type;
	double      latitude;			    // latitude - WGS84 (deg)
	double      longitude;			    // longitude - WGS84 (deg)
	double      height;				    // Ellipsoidal height - WGS84 (m)
	double      north_velocity;		    // velocity in a northerly direction (m/s)
	double      east_velocity;		    // velocity in an easterly direction (m/s)
	double      up_velocity;            // velocity in an up direction
	double      roll;				    // right handed rotation around y-axis (degrees)
	double      pitch;				    // right handed rotation aruond x-axis (degrees)
	double      heading;                // right handed rotation around z-axis (degrees)
    float       latitude_std;
	float       longitude_std;
	float       height_std;
	float       north_velocity_std;
	float       east_velocity_std;
	float       up_velocity_std;
	float       roll_std;
	float       pitch_std;
	float       heading_std;
} output_inspvax_struct;

typedef struct {
	uint16_t    gps_week;               // GPS Week number
    uint32_t    gps_millisecs;          // Milliseconds into week
    uint8_t     mode;
    double      speed;
    uint8_t     fwd;
    uint64_t    wheel_tick;
} output_odo_speed_struct;

typedef struct {
	uint16_t    gps_week;               // GPS Week number
    uint32_t    gps_millisecs;          // Milliseconds into week
    uint8_t     satelliteId;
    uint8_t     systemId;
    uint8_t     antennaId;
    uint8_t     l1cn0;
    uint8_t     l2cn0;
    float       azimuth;
    float       elevation;
} output_satellites_struct;


typedef struct {
/* IMU */
uint32_t imu_temp_status: 1; // imu temperature status
uint32_t imu_acce_status: 1;  // imu accelerometer status
uint32_t imu_gyro_status: 1; // imu gyro status

uint32_t imu_sensor_status1: 1; // imu sensor (#1, #2， #3) status
uint32_t imu_sensor_status2: 1; // imu sensor (#1, #2， #3) status
uint32_t imu_sensor_status3: 1; // imu sensor (#1, #2， #3) status
uint32_t imu_overall_status: 1;
/* GNSS status */
uint32_t gnss_data_status: 1; 
uint32_t gnss_signal_status: 1;
/* operation */
uint32_t power: 1; // for the whole device, any component has no power then 0
uint32_t MCU_status: 1;
uint32_t rexerved: 21;
} status_bit_t;
typedef struct 
{
    status_bit_t status_bit;
    float imu_temp;
    float mcu_temp;
}status_t;

#pragma pack()

extern status_t g_status;

int checkUserInPacketType(uint16_t receivedCode);
int checkUserOutPacketType(uint16_t receivedCode);
void user_inpacket_type_to_bytes(uint8_t bytes[]);


int  HandleUserInputPacket (UcbPacketStruct *ptrUcbPacket);

void send_rawimu_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket);
void send_bestgnss_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket, gnss_solution_t *const ptr_bestgnss_sol);
void send_inspvax_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket, ins_solution_t *const ptr_ins_sol);
void send_odospeed_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket);
void send_satellites_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket, gnss_solution_t *const ptr_sta);



#endif /* _USER_MESSAGE_H */
