
/** ***************************************************************************
 * @file   UserMessaging.h
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

#ifndef _USER_MESSAGING_SPI_H
#define _USER_MESSAGING_SPI_H

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "GlobalConstants.h"


typedef struct{
    uint16_t status;
    int16_t  rates[3];
    int16_t  accels[3];
    int16_t  temp; 
}spi_burst_data_t;

typedef struct{
    uint16_t status;
    int16_t  rates[3];
    int16_t  accels[3];
    int16_t  temp; 
    int16_t  sensToPps;
    int16_t  ppsToDrdy;
}ext_spi_burst_data_t;

#define  SPI_DATA_BUF_LEN    32
#define  SPI_BURST_DATA_SIZE 16
#define  NUM_SPI_REGS        128

// functions of SPI interface registers
#define SPI_REG_DRDY_RATE_CTRL         0x37     // DATA READY PIN output frequency divider (from 200Hz)
#define SPI_REG_ORIENTATION_LSB_CTRL   0x75     // Orientation LSB
#define SPI_REG_ORIENTATION_MSB_CTRL   0x74     // Orientation MSB
#define SPI_REG_ACCEL_FILTER_TYPE_CTRL 0x38     // Built-in LPF selection
#define SPI_REG_RATE_FILTER_TYPE_CTRL  0x39     // Built-in LPF selection
#define SPI_REG_MAG_ALIGN_CTRL         0x50     // mag alignment command
#define SPI_REG_MAG_ALIGN_CTRL2        0x51    // mag alignment command/status


#define SPI_REG_BURST_MSG_REQUEST      0x3E     // burst message request
#define SPI_REG_EXT_BURST_MSG_REQUEST  0x3F     // estended burst message request
#define SPI_REG_MANUF_CODE_REQUEST     0x52     // manufacturer code
#define SPI_REG_MANUF_LOC_REQUEST      0x53     // manufacturel location
#define SPI_REG_UNIT_CODE_REQUEST      0x54     // unit code
#define SPI_REG_PROD_ID_REQUEST        0x56     // product ID - 0x3830(default)
#define SPI_REG_SERIAL_NUM_REQUEST     0x58     // serial number request 
#define SPI_REG_MASTER_STATUS_REQUEST  0x5A     // master status request 
#define SPI_REG_HW_VERSION_REQUEST     0x7E     // HW version request 
#define SPI_REG_SW_VERSION_REQUEST     0x7F     // SW version request  

#define SPI_REG_HARD_IRON_BIAS_X_MSB   0x48     // Estimated hard iron bias X 
#define SPI_REG_HARD_IRON_BIAS_X_LSB   0x49     // Estimated hard iron bias X 
#define SPI_REG_HARD_IRON_BIAS_Y_MSB   0x4A     // Estimated hard iron bias Y 
#define SPI_REG_HARD_IRON_BIAS_Y_LSB   0x4B     // Estimated hard iron bias Y 
#define SPI_REG_SOFT_IRON_RATIO_MSB    0x4C     // Estimated soft iron ratio 
#define SPI_REG_SOFT_IRON_RATIO_LSB    0x4D     // Estimated soft iron ratio 
#define SPI_REG_SOFT_IRON_ANGLE_MSB    0x4E     // Estimated soft iron angle 
#define SPI_REG_SOFT_IRON_ANGLE_LSB    0x4F     // Estimated soft iron angle 

// options for DATA READY signal divider
enum output_drdy_rate {
    OUTPUT_DATA_RATE_ZERO   =   0,
    OUTPUT_DATA_RATE_200_HZ =   1,
    OUTPUT_DATA_RATE_100_HZ =   2,
    OUTPUT_DATA_RATE_50_HZ  =   4,
    OUTPUT_DATA_RATE_25_HZ  =   8,
    OUTPUT_DATA_RATE_20_HZ  =  10,
    OUTPUT_DATA_RATE_10_HZ  =  20,
    OUTPUT_DATA_RATE_5_HZ   =  40,
    OUTPUT_DATA_RATE_4_HZ   =  50,
    OUTPUT_DATA_RATE_2_HZ   = 100,
    OUTPUT_DATA_RATE_1_HZ   = 200
};

#define SENSOR_OVER_RANGE_STATUS_MASK  0x0010 

BOOL     loadSPIBurstData(spi_burst_data_t *data);
void     InitUserSPIRegisters();
int16_t  prepare_sensor_value( float   dataIn, float limit, float scaleFactor);
uint16_t GetSpiFilterType();
int      GetSpiPacketRateDivider();
float    GetSpiAccelScaleFactor();
float    GetSpiRateScaleFactor();
float    GetSpiAccelLimit();
float    GetSpiRateLimit();
void     fillSPIDataBuffer();
int      UserSPIPrepareForDataTransmit(uint8_t *out, int outLen);



// legacy stuff
#define USER_PACKET_OK      0
#define UNKNOWN_USER_PACKET 1
#define USER_PACKET_ERROR   2

#endif
