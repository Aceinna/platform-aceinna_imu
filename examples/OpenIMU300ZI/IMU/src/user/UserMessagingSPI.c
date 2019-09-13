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
#include "GlobalConstants.h"
#include "UserMessagingSPI.h"
#include "UserConfiguration.h"
#include "filter.h"
#include "platformAPI.h"
#include "boardAPI.h"
#include "sensorsAPI.h"
#include "spiAPI.h"


uint8_t  _spiDataBuf[2][SPI_DATA_BUF_LEN];
uint8_t  *_activeSpiDataBufPtr = _spiDataBuf[0];
uint8_t  _emptySpiDataBufIdx   = 0;
uint8_t  _activeSpiDataBufIdx  = 0;


uint8_t  _spiRegs[NUM_SPI_REGS];

BOOL     _readOp  = FALSE;
uint8_t  _regAddr = 0; 

// Initialize SPI registers here
// For default register assignments refer to 
// UserMessaging.h

/// initialise SPI geristers
void InitUserSPIRegisters()
{
   uint32_t  SN  = unitSerialNumber();
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
    _spiRegs[SPI_REG_MANUF_CODE_REQUEST]     = (arr[0] << 4) | arr[1]; 
    _spiRegs[SPI_REG_MANUF_LOC_REQUEST]      = (arr[2] << 4) | arr[3];
    _spiRegs[SPI_REG_UNIT_CODE_REQUEST]      = arr[4];
    _spiRegs[SPI_REG_UNIT_CODE_REQUEST+1]    = (arr[5] << 4 ) | arr[6];
    _spiRegs[SPI_REG_PROD_ID_REQUEST]        = 0x30;    // product code HI
    _spiRegs[SPI_REG_PROD_ID_REQUEST+1]      = 0x00;    // product code LO
    _spiRegs[SPI_REG_SERIAL_NUM_REQUEST]     = arr[7];
    _spiRegs[SPI_REG_SERIAL_NUM_REQUEST+1]   = (arr[8] << 4) | arr[9];
    // Fill up HW/SW versions
    _spiRegs[SPI_REG_HW_VERSION_REQUEST]     = ReadUnitHwConfiguration();
    _spiRegs[SPI_REG_SW_VERSION_REQUEST]     = unitSpiSwVersion();
    _spiRegs[SPI_REG_ACCEL_FILTER_TYPE_CTRL] = IIR_05HZ_LPF;
    _spiRegs[SPI_REG_RATE_FILTER_TYPE_CTRL]  = IIR_05HZ_LPF;
}


// Here  SPI burst packet contents get loaded into the buffers
// Each  new data set get loaded into new buffer, while pervious
// data set remains in old buffer. After loading active buffer get switched
// When user requests data from outside - dtata from active buffer will be transmitted
BOOL loadSPIBurstData(ext_spi_burst_data_t *data)
{
    if(sizeof(ext_spi_burst_data_t) > SPI_DATA_BUF_LEN){
        while(1);  // catch it here
    }
    memcpy(_spiDataBuf[_emptySpiDataBufIdx], data, sizeof(ext_spi_burst_data_t));
    _activeSpiDataBufPtr = _spiDataBuf[_emptySpiDataBufIdx];
    _activeSpiDataBufIdx = _emptySpiDataBufIdx;   //active buffer;
    _emptySpiDataBufIdx ^= 1;   //new buffer;
    return TRUE;
}

int  burstCnt = 0;
int  bufIdx[256];
uint8_t bufPtr = 0;

// Here SPI interface command (first byte of 16-bit transaction word)
// is processed based upon read or write request
// This interface is completely customizable
// In default (example):
// - read request with register access 0x3e forces unit to prepare burst packet 
//   for transmission (see description of the spi_burst_data_t structure)
// - write request to any register just assigns register address which will be used
//   in the processSPIData function 
void SPI_ProcessCommand(uint8_t cmd)
{
    static uint8_t  regData[2] = {0,0};
    _readOp  = (cmd & 0x80) == 0;
    _regAddr = 0xff;  // reset reg addr    
    // command in cmd"
    // place result in "out", place len into *len
    if(_readOp){
      switch(cmd & 0x7F){
        case 0x3e:
            // sensors data
            burstCnt++;
            bufIdx[bufPtr++] = _activeSpiDataBufIdx;
            UserSPIPrepareForDataTransmit(_activeSpiDataBufPtr, sizeof(spi_burst_data_t));
            break;
        case 0x3f:
            // ext sensors data
            burstCnt++;
            bufIdx[bufPtr++] = _activeSpiDataBufIdx;
            UserSPIPrepareForDataTransmit(_activeSpiDataBufPtr, sizeof(ext_spi_burst_data_t));
            break;
        default:
            // register data
            regData[0] = _spiRegs[cmd & 0x7F];
            regData[1] = _spiRegs[(cmd & 0x7F)+1];
            _regAddr   = 0xff; 
            UserSPIPrepareForDataTransmit(regData, 2);
            break;
      }
    }else{
      _regAddr = cmd & 0x7f; 
    }
}


BOOL CheckSpiSensorFilterType(uint8_t type)
{
    switch(type){
      case UNFILTERED:
      case IIR_05HZ_LPF:
      case IIR_10HZ_LPF:
      case IIR_20HZ_LPF:
      case IIR_25HZ_LPF:
      case IIR_40HZ_LPF:
      case IIR_50HZ_LPF:
            return TRUE;
        default:
            return FALSE; 
    }
};


// Here SPI write transaction is processed
// based on register address assigned in processSPICommand function
// data is demultiplexed to specific handlers
// handling of write transactions is can be changed at user discretion
// except of the case when unit orientation and sensors data filtering
// managed by the platform

void SPI_ProcessData(uint8_t* in)
{
    uint16_t tmp;
    static uint8_t orientShadow = 0xff;

    if(!_readOp || _regAddr != 0xff){
        switch(_regAddr){
        // Place read only registers here
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
        case SPI_REG_ORIENTATION_MSB_CTRL:
            orientShadow  = in[1];
            break;
        case SPI_REG_ORIENTATION_LSB_CTRL:
            if(orientShadow != 0xff){
                tmp = (orientShadow << 8) | in[1];
                if(platformApplyOrientation(tmp)){
                    _spiRegs[SPI_REG_ORIENTATION_MSB_CTRL] = orientShadow;
                    _spiRegs[SPI_REG_ORIENTATION_LSB_CTRL] = in[1];
                }
                orientShadow = 0xff;
            }
            break;
        case SPI_REG_ACCEL_FILTER_TYPE_CTRL:
            if(CheckSpiSensorFilterType(in[1])){
                _spiRegs[SPI_REG_ACCEL_FILTER_TYPE_CTRL] = in[1];
                platformUpdateAccelFilterType(in[1]);
            }
            break;
        case SPI_REG_RATE_FILTER_TYPE_CTRL:
            if(CheckSpiSensorFilterType(in[1])){
                _spiRegs[SPI_REG_RATE_FILTER_TYPE_CTRL] = in[1];
                platformUpdateRateFilterType(in[1]);
            }
            break;
        default:
            _spiRegs[_regAddr] = in[1];
            break;
        }
    }
    _regAddr = 0xff;
}

int16_t swap16(int16_t data)
{
    return ((data << 8 ) & 0xff00) | ((data >> 8) & 0x00ff);
}

uint16_t overRange = 0;

/** ***************************************************************************
 * @name prepare_sensor_value
 * @brief Optio9nal function to provide spoecific scaling and limits for 
 *        sensors Data
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

// Fills SPI burst data buffer with latest samples 
void FillSPIBurstDataBuffer()
{
    ext_spi_burst_data_t data;
    double Rates[3], Accels[3], Mags[3]; 
    double temp;
    float  t1;

    GetRateData_degPerSec(Rates);
    GetAccelData_g(Accels);
    GetBoardTempData(&temp);
    GetMagData_G(Mags);

    data.status = 0;
    overRange   = 0;
    
    for(int i = 0; i < 3; i++){
        data.accels[i] = prepare_sensor_value(Accels[i], GetSpiAccelLimit(), GetSpiAccelScaleFactor());
        data.rates[i]  = prepare_sensor_value(Rates[i], GetSpiRateLimit(), GetSpiRateScaleFactor());
        t1 = Mags[i] * 16384;
        data.mags[i]   = swap16((int16_t)t1);
    }

    if(overRange){
        data.status |= SENSOR_OVER_RANGE_STATUS_MASK;
    }
    t1 = temp;
    data.temp   = swap16((int16_t)t1);
    loadSPIBurstData(&data);
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
    return 100;
}

float  GetSpiRateLimit()
{
    return 300.0;
}

float    GetSpiAccelLimit()
{
    return 4.5;     // so far limited to 4,5G
}
