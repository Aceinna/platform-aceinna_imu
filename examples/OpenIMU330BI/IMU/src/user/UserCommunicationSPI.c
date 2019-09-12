/** ***************************************************************************
 * @file   UserCommunicationSPI.c
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
#include <math.h>
#include "GlobalConstants.h"
#include "UserCommunicationSPI.h"
#include "configurationAPi.h"
#include "calibrationAPI.h"
#include "sensorsAPI.h"
#include "filter.h"
#include "appVersion.h"
#include "hwAPI.h"


int16_t prepare_sensor_value( float   dataIn, float limit, float scaleFactor);



uint8_t  _spiDataBuf[2][SPI_DATA_BUF_LEN];
uint8_t  *_activeSpiDataBufPtr = _spiDataBuf[0];
uint8_t  _emptySpiDataBufIdx  = 0;


uint8_t  _spiRegs[NUM_SPI_REGS];

BOOL     _readOp  = FALSE;
uint8_t  _regAddr = 0; 

void InitUserCommunicationSPI()
{
   uint32_t  SN  = GetUnitSerialNum();
   uint32_t  div = 1000000000;
   uint8_t   arr[12];

   for(int i = 0; i < SPI_DATA_BUF_LEN; i++){
      _spiDataBuf[0][i] = 0;
      _spiDataBuf[1][i] = 0;
    }

   for(int i = 0; i < 128; i++){
      _spiRegs[i]  = 0;
    }

    for(int i = 0; i < 10; i++){
        arr[i] = SN/div;
        SN    %= div;
        div   /= 10;
    }
    // Fill up serial number and init IDs
    _spiRegs[SPI_REG_MANUF_CODE_REQUEST]   = (arr[0] << 4) | arr[1];
    _spiRegs[SPI_REG_MANUF_LOC_REQUEST]    = (arr[2] << 4) | arr[3];
    _spiRegs[SPI_REG_UNIT_CODE_REQUEST]    = arr[4];
    _spiRegs[SPI_REG_UNIT_CODE_REQUEST+1]  = (arr[5] << 4 ) | arr[6];
    _spiRegs[SPI_REG_PROD_ID_REQUEST]        = 0x33;                      // IMU330
    _spiRegs[SPI_REG_PROD_ID_REQUEST+1]      = 0x00;
    _spiRegs[SPI_REG_SERIAL_NUM_REQUEST]   = arr[7];
    _spiRegs[SPI_REG_SERIAL_NUM_REQUEST+1] = (arr[8] << 4) | arr[9];
    // fill up orientation
    _spiRegs[SPI_REG_ORIENTATION_LSB_CTRL] = 0x6B; // 117  -X -Y -Z
    _spiRegs[SPI_REG_ORIENTATION_MSB_CTRL] = 0x00; // 116
    // Fill up default DRDY rate
    _spiRegs[SPI_REG_DRDY_RATE_CTRL]         = 1;                         // 55
    _spiRegs[SPI_REG_DRDY_CTRL]              = 0x04;                      // 52
    _spiRegs[SPI_REG_FILTER_TYPE_CTRL]       = FIR_05HZ_LPF;              // 56
    _spiRegs[SPI_REG_RATE_SENSOR_RANGE_CTRL] = SPI_RATE_SENSOR_RANGE_125; // 57
    // Fill up default sensor selection
    _spiRegs[SPI_REG_CHIP1_SENSORS_CTRL]     = 0xFF;  // all sensors selected
    _spiRegs[SPI_REG_CHIP2_SENSORS_CTRL]     = 0xFF;  // all sensors selected
    _spiRegs[SPI_REG_CHIP3_SENSORS_CTRL]     = 0xFF;  // all sensors selected
    // Fill up HW/SW versions
    _spiRegs[SPI_REG_HW_VERSION_REQUEST]     = HW_ReadConfig();     // 126
    _spiRegs[SPI_REG_SW_VERSION_REQUEST]     = SPI_SW_VERSION;      // 127

//  Apply filter type 
//  configSetSensorFilterTypeForSPI(FIR_05HZ_LPF);  // 5Hz

    SPI_ActivateInterface();
}


BOOL LoadSPIBurstData(spi_burst_data_t *data)
{
    if(sizeof(spi_burst_data_t) > SPI_DATA_BUF_LEN){
        return FALSE;
    }
    memcpy(_spiDataBuf[_emptySpiDataBufIdx], data, sizeof(spi_burst_data_t));
    _activeSpiDataBufPtr = _spiDataBuf[_emptySpiDataBufIdx];
    _emptySpiDataBufIdx ^= 1;   //new buffer;
    return TRUE;
}

int SPI_ProcessCommand(uint8_t cmd)
{
    static uint8_t  regData[2] = {0,0};
    _readOp  = (cmd & 0x80) == 0;
    _regAddr = 0xff;  // reset reg addr    
    // command in cmd"
    // place result in "out", place len into *len
    if(_readOp){
      switch(cmd & 0x7F){
        case 0x3e:
            // Prepare to transmit burst sensors data packet
            SPI_PrepareForDataTransmit(_activeSpiDataBufPtr, sizeof(spi_burst_data_t));
            break;
        default:
            // Prepare to transmit contents of SPI register
            regData[0] = _spiRegs[cmd & 0x7F];
            regData[1] = _spiRegs[(cmd & 0x7F)+1];
            _regAddr   = 0xff; 
            SPI_PrepareForDataTransmit(regData, 2);
            break;
      }
    }else{
      _regAddr = cmd & 0x7f; 
    }
    
    return 0;
}

BOOL CheckSpiPacketRateDivider(uint8_t dataRate )
{
    switch( dataRate )
    {
        case OUTPUT_DATA_RATE_ZERO  :
        case OUTPUT_DATA_RATE_200_HZ:
        case OUTPUT_DATA_RATE_100_HZ:
        case OUTPUT_DATA_RATE_50_HZ :
        case OUTPUT_DATA_RATE_25_HZ :
        case OUTPUT_DATA_RATE_20_HZ :
        case OUTPUT_DATA_RATE_10_HZ :
        case OUTPUT_DATA_RATE_5_HZ  :
        case OUTPUT_DATA_RATE_4_HZ  :
        case OUTPUT_DATA_RATE_2_HZ  :
        case OUTPUT_DATA_RATE_1_HZ  :
            return TRUE;
            break;
        default:
            return FALSE;
            break;
    }
} /* end CheckPacketRateDivider */

BOOL CheckSpiRateSensorRange(uint8_t range )
{
    switch( range )
    {
        case SPI_RATE_SENSOR_RANGE_62P5 :
        case SPI_RATE_SENSOR_RANGE_125  :
        case SPI_RATE_SENSOR_RANGE_250  :
        case SPI_RATE_SENSOR_RANGE_500  :
        case SPI_RATE_SENSOR_RANGE_1000 :
            return TRUE;
            break;
        default:
            return FALSE;
            break;
    }
} /* end CheckPacketRateDivider */


BOOL CheckSpiSensorFilterType(uint8_t type)
{
    switch(type){
      case UNFILTERED:
      case FIR_40HZ_LPF:
      case FIR_20HZ_LPF:
      case FIR_10HZ_LPF:
      case FIR_05HZ_LPF:
      case IIR_50HZ_LPF:
      case IIR_20HZ_LPF:
      case IIR_10HZ_LPF:
      case IIR_05HZ_LPF:
            return TRUE;
        default:
            return FALSE; 
    }
};


void SPI_ProcessData(uint8_t* in)
{
    int chipIdx;
    uint16_t tmp;
    static uint8_t orientShadow = 0xff;

    if(!_readOp || _regAddr != 0xff){
        switch(_regAddr){
        // Place read only registers here
        // Do not do anything
        case SPI_REG_MANUF_CODE_REQUEST:
        case SPI_REG_MANUF_LOC_REQUEST:
        case SPI_REG_UNIT_CODE_REQUEST:
        case SPI_REG_UNIT_CODE_REQUEST+1:
        case SPI_REG_PROD_ID_REQUEST:
        case SPI_REG_PROD_ID_REQUEST+1:
        case SPI_REG_SERIAL_NUM_REQUEST:
        case SPI_REG_SERIAL_NUM_REQUEST+1:
        case SPI_REG_HW_VERSION_REQUEST:
        case SPI_REG_SW_VERSION_REQUEST:
            break;
        case SPI_REG_CHIP1_SENSORS_CTRL:
        case SPI_REG_CHIP2_SENSORS_CTRL:
        case SPI_REG_CHIP3_SENSORS_CTRL:
            _spiRegs[_regAddr] = in[1];
            chipIdx = _regAddr - SPI_REG_CHIP1_SENSORS_CTRL;
            configSetUsedSensors(chipIdx, in[1]);
            break;
        case SPI_REG_RATE_SENSOR_RANGE_CTRL:
            if(CheckSpiRateSensorRange(in[1])){
                _spiRegs[_regAddr] = in[1];
            }
            break;
        case SPI_REG_ORIENTATION_MSB_CTRL:
            orientShadow  = in[1];
            break;
        case SPI_REG_ORIENTATION_LSB_CTRL:
            if(orientShadow != 0xff){
                tmp = (orientShadow << 8) | in[1];
                if(configApplyOrientation(tmp)){
                    _spiRegs[SPI_REG_ORIENTATION_MSB_CTRL] = orientShadow;
                    _spiRegs[SPI_REG_ORIENTATION_LSB_CTRL] = in[1];
                }
                orientShadow = 0xff;
            }
            break;
        case SPI_REG_FILTER_TYPE_CTRL:
        if(CheckSpiSensorFilterType(in[1])){
            _spiRegs[_regAddr] = in[1];
            configSetSensorFilterTypeForSPI(in[1]);
        }
        break;
        default:
            // just write data into register
            _spiRegs[_regAddr] = in[1];
            break;
        }
    }
    _regAddr = 0xff;
}

uint16_t GetSpiFilterType()
{
    return _spiRegs[SPI_REG_FILTER_TYPE_CTRL];
}

int16_t swap16(int16_t data)
{
    return ((data << 8 ) & 0xff00) | ((data >> 8) & 0x00ff);
}

uint16_t overRange = 0;
/** ***************************************************************************
 * @name _loadRegisters()
 * @brief convert float values from the gAgorithm data structure scaled and cast
 *        to to an short then split to bytes to load a pair of SPI "registers"
 *        after data collection.
 *
 * @param [in] dataIn floating point data to be processed
 * @param [in] floatLimit input data limit
 * @param [in] scale factor to scale the input value to a signed short
 * @param [in] buffer - the SPI register to be filled
 * @param [in] index - where in the spi register to put the data
 * @retval N/A
 ******************************************************************************/
int16_t prepare_sensor_value( float   dataIn, float limit, float scaleFactor)
{
    static uint16_t UpperLimit = 32760;
    float           tempFloat  = 0.0;
    int             sign;
    static int16_t  tempInt16  = 0x0000;

    tempFloat = dataIn;
    if( tempFloat > limit ) {
        tempFloat = limit;
        overRange = 1;
    } else if( tempFloat < -limit ) {
        tempFloat = -limit;
        overRange = 1;
    }

    if( tempFloat >= 0 ) {
      sign = 1;
    } else {
      sign = -1;
    }

    // now tempInt16 can be positive:
    tempInt16 = ( int )( sign * tempFloat * scaleFactor );
    if( tempInt16 > UpperLimit ) {
        overRange = 1;
        tempInt16 = UpperLimit;
    }

    tempInt16 = sign * tempInt16;

    tempInt16 = (( tempInt16 & 0xFF00 ) >> 8 ) | ((tempInt16 & 0x00FF ) << 8);

    return tempInt16;

}

void FillSPIDataBuffer()
{
    spi_burst_data_t data;
    double accels[3];
    double rates[3];
    double temp;
    
    GetAccelData_g(accels);
    GetRateData_degPerSec(rates);

    data.status = 0;
    overRange   = 0;
    for(int i = 0; i < 3; i++){
        data.accels[i] = prepare_sensor_value((float)accels[i], GetSpiAccelLimit(), GetSpiAccelScaleFactor());
        data.rates[i]  = prepare_sensor_value((float)rates[i], GetSpiRateLimit(), GetSpiRateScaleFactor());
    }
    if(overRange){
        data.status |= SENSOR_OVER_RANGE_STATUS_MASK;
    }
    
    temp        = GetUnitTemp();
    temp        = (temp - 31.0)/0.073111172849435;
    data.temp   = swap16((int16_t)temp);

    LoadSPIBurstData(&data);
}

int  GetSpiPacketRateDivider()
{
    return _spiRegs[SPI_REG_DRDY_RATE_CTRL];
}

float GetSpiAccelScaleFactor()
{
    return 4000.0;  // so far limited to 5G        
}

float GetSpiRateScaleFactor()
{
    return 400/_spiRegs[SPI_REG_RATE_SENSOR_RANGE_CTRL];
}

float    GetSpiRateLimit()
{
    switch(_spiRegs[SPI_REG_RATE_SENSOR_RANGE_CTRL]){
        case SPI_RATE_SENSOR_RANGE_62P5 :   return 62.5;
        case SPI_RATE_SENSOR_RANGE_125  :   return 125;
        case SPI_RATE_SENSOR_RANGE_250  :   return 250;
        case SPI_RATE_SENSOR_RANGE_500  :   return 500;
        case SPI_RATE_SENSOR_RANGE_1000 :   return 1000;
        default: return 125; 
    }
}

float    GetSpiAccelLimit()
{
    return 4.5;     // so far limited to 4,5G
}

