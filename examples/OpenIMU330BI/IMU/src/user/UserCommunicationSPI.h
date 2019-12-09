/** ***************************************************************************
 * @file   UserCommunicationSPI.h
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

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "GlobalConstants.h"
#ifndef _USER_COM_SPI
#define _USER_COM_SPI


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

#define SPI_REG_RESET_LSB_CTRL          0x02
#define SPI_REG_RESET_MSB_CTRL          0x03

#define SPI_REG_DRDY_RATE_CTRL         0x37
#define SPI_REG_ACCEL_FILTER_TYPE_CTRL  0x38
#define SPI_REG_ACCEL_SENSOR_RANGE_CTRL 0x70
#define SPI_REG_RATE_SENSOR_RANGE_CTRL  0x71
#define SPI_REG_MAG_SENSOR_RANGE_CTRL   0x72
#define SPI_REG_ORIENTATION_LSB_CTRL   0x75
#define SPI_REG_ORIENTATION_MSB_CTRL   0x74
#define SPI_REG_SAVE_CFG_CTRL           0x76
#define SPI_REG_RATE_FILTER_TYPE_CTRL   0x78

#define SPI_REG_SELF_TEST_CTRL         0x35
#define SPI_REG_CLOB_CTRL              0x3E
#define SPI_REG_DRDY_CTRL              0x34


#define SPI_REG_XRATE_REQUEST           0x04
#define SPI_REG_YRATE_REQUEST           0x06
#define SPI_REG_ZRATE_REQUEST           0x08
#define SPI_REG_XACCEL_REQUEST          0x0A
#define SPI_REG_YACCEL_REQUEST          0x0C
#define SPI_REG_ZACCEL_REQUEST          0x0E
#define SPI_REG_RTEMP_REQUEST           0x16
#define SPI_REG_BTEMP_REQUEST           0x18

#define SPI_REG_BURST_MSG_REQUEST      0x3E



#define SPI_REG_MAG_SENSOR_SCALE        0x32
#define SPI_REG_ACCEL_SENSOR_SCALE      0x46
#define SPI_REG_RATE_SENSOR_SCALE       0x47
#define SPI_REG_MANUF_CODE_REQUEST     0x52
#define SPI_REG_MANUF_LOC_REQUEST      0x53
#define SPI_REG_UNIT_CODE_REQUEST      0x54
#define SPI_REG_PROD_ID_REQUEST        0x56     // 0x3830
#define SPI_REG_SERIAL_NUM_REQUEST     0x58
#define SPI_REG_MASTER_STATUS_REQUEST  0x5A
#define SPI_REG_HW_STATUS_REQUEST       0x5C
#define SPI_REG_SW_STATUS_REQUEST       0x5E
#define SPI_REG_HW_VERSION_REQUEST     0x7E
#define SPI_REG_SW_VERSION_REQUEST     0x7F

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

enum output_rate_sensor_dyn_range{
    SPI_RATE_SENSOR_RANGE_500  = 0x08,
    SPI_RATE_SENSOR_RANGE_1000 = 0x10,
    SPI_RATE_SENSOR_RANGE_2000 = 0x20,
};

enum output_accel_sensor_dyn_range{
    SPI_ACCEL_SENSOR_RANGE_8G   = 8,
    SPI_ACCEL_SENSOR_RANGE_16G  = 16,
};


#define  ACCEL_SENSOR_OVER_RANGE_STATUS_MASK  0x0010 
#define  RATE_SENSOR_OVER_RANGE_STATUS_MASK   0x0020 
#define  HW_FAILURE_MASK                      0x0002 
#define  SW_FAILURE_MASK                      0x0004 
#define  SENSOR_FAILURE_MASK                  0x1000 


BOOL     LoadSPIBurstData(spi_burst_data_t *data);
void     InitUserCommunicationSPI();
float    GetSpiAccelScaleFactor(uint8_t *out);
float    GetSpiRateScaleFactor(uint8_t *out);
float    GetSpiAccelLimit();
float    GetSpiRateLimit();
void     FillSPIDataBuffer();
int      GetSpiPacketRateDivider();
void     UpdateSpiUserConfig();
#endif
