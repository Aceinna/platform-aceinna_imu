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
#include "BitStatus.h"
#include "UserConfiguration.h"


int16_t prepare_sensor_value( float   dataIn, float limit, float scaleFactor);



uint8_t  _spiDataBuf[2][SPI_DATA_BUF_LEN];
uint8_t  *_activeSpiDataBufPtr = _spiDataBuf[0];
uint8_t  _emptySpiDataBufIdx  = 0;


uint8_t  _spiRegs[NUM_SPI_REGS];

BOOL     _readOp  = FALSE;
uint8_t  _regAddr = 0; 
BOOL     fSaveSpiConfig = FALSE;

void InitUserCommunicationSPI()
{
   uint32_t  SN  = GetUnitSerialNum();
   uint32_t  div = 1000000000;
   uint8_t   arr[12];
   uint16_t  tmp;
   uint8_t   asf, gsf;

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
    tmp = SpiOrientation();
    _spiRegs[SPI_REG_ORIENTATION_LSB_CTRL]    = tmp & 0xff;                   
    _spiRegs[SPI_REG_ORIENTATION_MSB_CTRL]    = (tmp >> 8) & 0xff;
    // Fill up default DRDY rate
    _spiRegs[SPI_REG_DRDY_RATE_CTRL]          = SpiSyncRate();  
    _spiRegs[SPI_REG_DRDY_CTRL]               = 0x04;            
    _spiRegs[SPI_REG_ACCEL_FILTER_TYPE_CTRL]  = SpiAccelLpfType();     
    _spiRegs[SPI_REG_RATE_FILTER_TYPE_CTRL]   = SpiGyroLpfType();           
    _spiRegs[SPI_REG_ACCEL_SENSOR_RANGE_CTRL] = configGetAccelRange();
    _spiRegs[SPI_REG_RATE_SENSOR_RANGE_CTRL]  = configGetGyroRange()*8/500;
    
     GetSpiAccelScaleFactor(&asf);                       
     GetSpiRateScaleFactor(&gsf);                       
    
    _spiRegs[SPI_REG_ACCEL_SENSOR_SCALE]      = asf;                       
    _spiRegs[SPI_REG_RATE_SENSOR_SCALE]       = gsf;

    // Fill up HW/SW versions
    _spiRegs[SPI_REG_HW_VERSION_REQUEST]     = HW_ReadConfig();     // 126
    _spiRegs[SPI_REG_SW_VERSION_REQUEST]     = SPI_SW_VERSION;      // 127


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
// Accelerometer data
    _spiRegs[SPI_REG_XACCEL_REQUEST]   = data->accels[0] & 0xff;
    _spiRegs[SPI_REG_XACCEL_REQUEST+1] = (data->accels[0] >> 8) & 0xff;
    _spiRegs[SPI_REG_YACCEL_REQUEST]   = data->accels[1] & 0xff;
    _spiRegs[SPI_REG_YACCEL_REQUEST+1] = (data->accels[1] >> 8) & 0xff;
    _spiRegs[SPI_REG_ZACCEL_REQUEST]   = data->accels[2] & 0xff;
    _spiRegs[SPI_REG_ZACCEL_REQUEST+1] = (data->accels[2] >> 8) & 0xff;
// Gyro data
    _spiRegs[SPI_REG_XRATE_REQUEST]   = data->rates[0] & 0xff;
    _spiRegs[SPI_REG_XRATE_REQUEST+1] = (data->rates[0] >> 8) & 0xff;
    _spiRegs[SPI_REG_YRATE_REQUEST]   = data->rates[1] & 0xff;
    _spiRegs[SPI_REG_YRATE_REQUEST+1] = (data->rates[1] >> 8) & 0xff;
    _spiRegs[SPI_REG_ZRATE_REQUEST]   = data->rates[2] & 0xff;
    _spiRegs[SPI_REG_ZRATE_REQUEST+1] = (data->rates[2] >> 8) & 0xff;
// Temp data    
    _spiRegs[SPI_REG_RTEMP_REQUEST]   = data->temp & 0xff;
    _spiRegs[SPI_REG_RTEMP_REQUEST+1] = (data->temp >> 8) & 0xff;
    _spiRegs[SPI_REG_BTEMP_REQUEST]   = data->temp & 0xff;
    _spiRegs[SPI_REG_BTEMP_REQUEST+1] = (data->temp >> 8) & 0xff;
// Master Status
    if(gBitStatus.hwBIT.all != 0){
        data->status |= HW_FAILURE_MASK;
        data->status |= SENSOR_FAILURE_MASK;
    }
    if(gBitStatus.swBIT.all != 0){
        data->status |= SW_FAILURE_MASK;
    }
    _spiRegs[SPI_REG_MASTER_STATUS_REQUEST]   = data->status & 0xff;
    _spiRegs[SPI_REG_MASTER_STATUS_REQUEST+1] = (data->status >> 8) & 0xff;
// HW Status
    _spiRegs[SPI_REG_HW_STATUS_REQUEST]    = gBitStatus.hwBIT.all & 0xff;
    _spiRegs[SPI_REG_HW_STATUS_REQUEST+1]  = (gBitStatus.hwBIT.all >> 8) & 0xff;
// SW Status
    _spiRegs[SPI_REG_SW_STATUS_REQUEST]    = gBitStatus.swBIT.all & 0xff;
    _spiRegs[SPI_REG_SW_STATUS_REQUEST+1]  = (gBitStatus.swBIT.all >> 8) & 0xff;

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

BOOL CheckSpiSyncRate(uint8_t rate)
{
    if(rate < 10){
        return TRUE;
    }
    return FALSE;
}

BOOL CheckSpiRateSensorRange(uint8_t range )
{
    switch( range )
    {
        case SPI_RATE_SENSOR_RANGE_500  :
        case SPI_RATE_SENSOR_RANGE_1000 :
        case SPI_RATE_SENSOR_RANGE_2000 :
            return TRUE;
            break;
        default:
            return FALSE;
            break;
    }
} /* end CheckPacketRateDivider */

BOOL CheckSpiAccelSensorRange(uint8_t range )
{
    switch( range )
    {
        case SPI_ACCEL_SENSOR_RANGE_8G   :
        case SPI_ACCEL_SENSOR_RANGE_16G  :
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

uint16_t GetSpiOrientationFromRegs()
{
    uint16_t tmp;
    
    tmp = (_spiRegs[SPI_REG_ORIENTATION_MSB_CTRL] << 8) | _spiRegs[SPI_REG_ORIENTATION_LSB_CTRL];
    return tmp;
}


void SaveParam(uint8_t paramId)
{
    switch(paramId){
        case 0:
        case 0xff:
            // save all parameters
            gUserConfiguration.spiOrientation  =  GetSpiOrientationFromRegs();
            gUserConfiguration.spiGyroLpfType  = _spiRegs[SPI_REG_RATE_FILTER_TYPE_CTRL];
            gUserConfiguration.spiAccelLpfType = _spiRegs[SPI_REG_ACCEL_FILTER_TYPE_CTRL];
            gUserConfiguration.gyroRange       = _spiRegs[SPI_REG_RATE_SENSOR_RANGE_CTRL];
            gUserConfiguration.accelRange      = _spiRegs[SPI_REG_ACCEL_SENSOR_RANGE_CTRL];
        case SPI_REG_DRDY_RATE_CTRL:
            gUserConfiguration.spiSyncRate     = _spiRegs[SPI_REG_DRDY_RATE_CTRL];
            break;
        case SPI_REG_ACCEL_FILTER_TYPE_CTRL:
            gUserConfiguration.spiAccelLpfType = _spiRegs[SPI_REG_ACCEL_FILTER_TYPE_CTRL];
            break;
        case SPI_REG_RATE_FILTER_TYPE_CTRL: 
            gUserConfiguration.spiGyroLpfType  = _spiRegs[SPI_REG_RATE_FILTER_TYPE_CTRL];
            break;
        case SPI_REG_ACCEL_SENSOR_RANGE_CTRL:
            gUserConfiguration.accelRange      = _spiRegs[SPI_REG_ACCEL_SENSOR_RANGE_CTRL];
            break;
        case SPI_REG_RATE_SENSOR_RANGE_CTRL:
            gUserConfiguration.gyroRange       = _spiRegs[SPI_REG_RATE_SENSOR_RANGE_CTRL];
            break;
        case SPI_REG_ORIENTATION_MSB_CTRL:
        case SPI_REG_ORIENTATION_LSB_CTRL:
            gUserConfiguration.spiOrientation  =  GetSpiOrientationFromRegs();
            break;
        default:
            return;
    }

    fSaveSpiConfig = TRUE;

}

void UpdateSpiUserConfig()
{
    if(fSaveSpiConfig){
        SaveUserConfig();
        fSaveSpiConfig = FALSE;
    }
}


void SPI_ProcessData(uint8_t* in)
{
    uint16_t tmp;
    static uint8_t orientShadow = 0xff;
    static uint8_t resetShadow  = 0x00;

    if(!_readOp || _regAddr != 0xff){
        switch(_regAddr){
        case SPI_REG_RESET_MSB_CTRL:
            if(in[1] == 0x55){
                resetShadow = 1;
            }else {
                resetShadow = 1;
            }
            break;
        case SPI_REG_RESET_LSB_CTRL:
            if(in[1] == 0xaa && resetShadow){
                HW_SystemReset();
            }else {
                resetShadow = 0;
            }
            break;
        case SPI_REG_RATE_SENSOR_RANGE_CTRL:
            if(CheckSpiRateSensorRange(in[1])){
                _spiRegs[_regAddr] = in[1];
            }
            break;
        case SPI_REG_DRDY_RATE_CTRL:
            if(CheckSpiSyncRate(in[1])){
                _spiRegs[_regAddr] = in[1];
            }
            break;
        case SPI_REG_ACCEL_SENSOR_RANGE_CTRL:
            if(CheckSpiAccelSensorRange(in[1])){
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
        case SPI_REG_ACCEL_FILTER_TYPE_CTRL:
        if(CheckSpiSensorFilterType(in[1])){
            _spiRegs[_regAddr] = in[1];
                configSetAccelSensorFilterTypeForSPI(in[1]);
        }
        break;
        case SPI_REG_RATE_FILTER_TYPE_CTRL:
            if(CheckSpiSensorFilterType(in[1])){
            _spiRegs[_regAddr] = in[1];
                configSetRateSensorFilterTypeForSPI(in[1]);
            }
            break;
        case SPI_REG_SAVE_CFG_CTRL:
            SaveParam(in[1]);
            break;
        default:
            break;
        }
    }
    if(_regAddr != SPI_REG_RESET_MSB_CTRL){
        resetShadow = 0;
    }
    _regAddr = 0xff;
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
    uint8_t dummy;
    
    GetAccelData_g(accels);
    GetRateData_degPerSec(rates);

    data.status = 0;
    overRange   = 0;
    for(int i = 0; i < 3; i++){
        data.accels[i] = prepare_sensor_value((float)accels[i], GetSpiAccelLimit(), GetSpiAccelScaleFactor(&dummy));
        if(overRange){
            data.status |= ACCEL_SENSOR_OVER_RANGE_STATUS_MASK;
            overRange = 0;
    }
        data.rates[i]  = prepare_sensor_value((float)rates[i], GetSpiRateLimit(), GetSpiRateScaleFactor(&dummy));
    if(overRange){
            data.status |= RATE_SENSOR_OVER_RANGE_STATUS_MASK;
            overRange = 0;
        }
    }
    
    temp        = GetUnitTemp();
    temp        = (temp - 31.0)/0.073111172849435;
    data.temp   = swap16((int16_t)temp);

    LoadSPIBurstData(&data);
}

int  GetSpiPacketRateDivider()
{
    switch(_spiRegs[SPI_REG_DRDY_RATE_CTRL]){
        case 0: return 0;   // 0 Hz
        case 2: return 2;   // 100 Hz
        case 3: return 4;   // 50 Hz
        case 4: return 8;   // 25 Hz
        case 5: return 10;  // 20 Hz
        case 6: return 20;  // 10 Hz
        case 7: return 40;  // 5 Hz
        case 8: return 50;  // 4 Hz
        case 9: return 100; // 2 Hz
        default:
            return 1;       // 200 Hz
    }
}

void SetSpiPacketRateDivider(uint8_t divider)
{
    _spiRegs[SPI_REG_DRDY_RATE_CTRL] = divider;
}


float GetSpiAccelScaleFactor(uint8_t *out)
{
    switch(_spiRegs[SPI_REG_ACCEL_SENSOR_SCALE]){
        case SPI_ACCEL_SENSOR_RANGE_16G  :   
            *out = 2;
            return 2000.0;
        default: 
            *out = 4;
            return 4000.0;
    }
}

float GetSpiRateScaleFactor(uint8_t *out)
{
    switch(_spiRegs[SPI_REG_RATE_SENSOR_SCALE]){
        case SPI_RATE_SENSOR_RANGE_1000 :   
            *out = 32;
            return 32.0;
        case SPI_RATE_SENSOR_RANGE_2000 :   
            *out = 16;
            return 16.0;
        default: 
            *out = 64;
            return 64.0; 
    }
}

float    GetSpiRateLimit()
{
    switch(_spiRegs[SPI_REG_RATE_SENSOR_RANGE_CTRL]){
        case SPI_RATE_SENSOR_RANGE_500  :   return 500;
        case SPI_RATE_SENSOR_RANGE_1000 :   return 1000;
        case SPI_RATE_SENSOR_RANGE_2000 :   return 2000;
        default: return 500; 
    }
}

float    GetSpiAccelLimit()
{
    switch(_spiRegs[SPI_REG_ACCEL_SENSOR_RANGE_CTRL]){
        case SPI_ACCEL_SENSOR_RANGE_16G  :   
            return 16;
        default: 
            return 8; 
    }
}

