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
#include "magAPI.h"
#include "appVersion.h"
#include "BitStatus.h"

uint8_t  _spiDataBuf[2][SPI_DATA_BUF_LEN];
uint8_t  *_activeSpiDataBufPtr = _spiDataBuf[0];
uint8_t  _emptySpiDataBufIdx   = 0;
uint8_t  _activeSpiDataBufIdx  = 0;


uint8_t  _spiRegs[NUM_SPI_REGS];

BOOL     _readOp  = FALSE;
uint8_t  _regAddr = 0; 
BOOL fSaveSpiConfig = FALSE;
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
    // fill up orientation
    uint16_t tmp = SpiOrientation();
    _spiRegs[SPI_REG_ORIENTATION_LSB_CTRL]    = tmp & 0xff;                   
    _spiRegs[SPI_REG_ORIENTATION_MSB_CTRL]    = (tmp >> 8) & 0xff;
    // Fill up default DRDY rate
    _spiRegs[SPI_REG_DRDY_RATE_CTRL]          = SpiSyncRate();  
    _spiRegs[SPI_REG_DRDY_CTRL]               = 0x04;            
    _spiRegs[SPI_REG_ACCEL_FILTER_TYPE_CTRL]  = SpiAccelLpfType();     
    _spiRegs[SPI_REG_RATE_FILTER_TYPE_CTRL]   = SpiGyroLpfType();           
    _spiRegs[SPI_REG_ACCEL_SENSOR_RANGE_CTRL] = SPI_ACCEL_SENSOR_RANGE_8G;
    _spiRegs[SPI_REG_RATE_SENSOR_RANGE_CTRL]  = SPI_RATE_SENSOR_RANGE_500;
    _spiRegs[SPI_REG_MAG_SENSOR_RANGE_CTRL]   = SPI_MAG_SENSOR_RANGE_2G;
    // Fill up sensors data scale factor on the SPI bus
    _spiRegs[SPI_REG_ACCEL_SENSOR_SCALE]      = 4;   // 4000 counts per 1 G -> 8G                       
    _spiRegs[SPI_REG_RATE_SENSOR_SCALE]       = 64;  // 64 counts   per dps -> 500 dps
    _spiRegs[SPI_REG_MAG_SENSOR_SCALE]        = 16;  // 16384 64 counts  per Gauss
    // Fill up HW/SW versions
    _spiRegs[SPI_REG_HW_VERSION_REQUEST]     = ReadUnitHwConfiguration();
    _spiRegs[SPI_REG_SW_VERSION_REQUEST]     = APP_SPI_SW_VERSION;
    {
        uint8_t regOffset;
        magAlignUserParams_t params;
        int16_t tmp[4];
        getUserMagAlignParams(&params);
        regOffset = SPI_REG_HARD_IRON_BIAS_X_MSB;
        tmp[0] = (int16_t)((params.hardIron_X/10) * 32768 );       // +-10 Gauss 
        tmp[1] = (int16_t)((params.hardIron_Y/10) * 32768 );       // +-10 Gauss 
        tmp[2] = (int16_t)(params.softIron_Ratio * 32768 );            //  
        tmp[3] = (int16_t)((params.softIron_Angle/3.141592) * 32768 ); // +-3.141592 rad 
        for(int i = 0; i < 4; i++){
            _spiRegs[regOffset++] = (tmp[i] >> 8) & 0xff;
            _spiRegs[regOffset++] = tmp[i] & 0xff;
        }
    }


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
// Mag data
    _spiRegs[SPI_REG_XMAG_REQUEST]   = data->mags[0] & 0xff;
    _spiRegs[SPI_REG_XMAG_REQUEST+1] = (data->mags[0] >> 8) & 0xff;
    _spiRegs[SPI_REG_YMAG_REQUEST]   = data->mags[1] & 0xff;
    _spiRegs[SPI_REG_YMAG_REQUEST+1] = (data->mags[1] >> 8) & 0xff;
    _spiRegs[SPI_REG_ZMAG_REQUEST]   = data->mags[2] & 0xff;
    _spiRegs[SPI_REG_ZMAG_REQUEST+1] = (data->mags[2] >> 8) & 0xff;
// Temp data    
    _spiRegs[SPI_REG_RTEMP_REQUEST]   = data->temp & 0xff;
    _spiRegs[SPI_REG_RTEMP_REQUEST+1] = (data->temp >> 8) & 0xff;
    _spiRegs[SPI_REG_BTEMP_REQUEST]   = data->temp & 0xff;
    _spiRegs[SPI_REG_BTEMP_REQUEST+1] = (data->temp >> 8) & 0xff;
// Master Status
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
        case SPI_REG_MAG_ALIGN_CTRL:
            {
                uint8_t state, regOffset;
                real    params[4];
                int16_t tmp[4];
                state = GetMagAlignEstimatedParams(params);
                _spiRegs[SPI_REG_MAG_ALIGN_CTRL2] = state;
                regOffset = SPI_REG_HARD_IRON_BIAS_X_MSB;
                tmp[0] = (int16_t)((params[0]/10) * 32768 );       // +-10 Gauss 
                tmp[1] = (int16_t)((params[1]/10) * 32768 );       // +-10 Gauss 
                tmp[2] = (int16_t)(params[2] * 32768 );            //  
                tmp[3] = (int16_t)((params[3]/3.141592) * 32767 ); // +-3.141592 rad 
                for(int i = 0; i < 4; i++){
                    _spiRegs[regOffset++] = (tmp[i] >> 8) & 0xff;
                    _spiRegs[regOffset++] = tmp[i] & 0xff;
                }
            }
            // fall through to send data
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


BOOL CheckSpiSensorFilterType(uint8_t type)
{
    switch(type){
      case UNFILTERED:
      case FIR_40HZ_LPF:
      case FIR_20HZ_LPF:
      case FIR_10HZ_LPF:
      case FIR_05HZ_LPF:
      case IIR_02HZ_LPF:
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

BOOL CheckSpiSyncRate(uint8_t rate)
{
    if(rate < 10){
        return TRUE;
    }
    return FALSE;
}

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
        case SPI_REG_DRDY_RATE_CTRL:
            gUserConfiguration.spiSyncRate     = _spiRegs[SPI_REG_DRDY_RATE_CTRL];
            break;
        case SPI_REG_ACCEL_FILTER_TYPE_CTRL:
            gUserConfiguration.spiAccelLpfType = _spiRegs[SPI_REG_ACCEL_FILTER_TYPE_CTRL];
            break;
        case SPI_REG_RATE_FILTER_TYPE_CTRL: 
            gUserConfiguration.spiGyroLpfType  = _spiRegs[SPI_REG_RATE_FILTER_TYPE_CTRL];
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
                Reset();
            }else {
                resetShadow = 0;
            }
            break;
        case SPI_REG_DRDY_RATE_CTRL:
            if(CheckSpiSyncRate(in[1])){
                _spiRegs[_regAddr] = in[1];
            }
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
        case SPI_REG_MAG_ALIGN_CTRL:
            {
                uint8_t len = 1;
		        ProcessMagAlignCmds((magAlignCmdPayload*)&in[1], &len);
            }
            break;
        case SPI_REG_SAVE_CFG_CTRL:
            SaveParam(in[1]);
            break;
        default:
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
    static uint16_t UpperLimit = 32766;
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
    t1 = (temp - 31.0)/0.073111172849435;
    data.temp   = swap16((int16_t)t1);
    loadSPIBurstData(&data);
}

float GetSpiAccelScaleFactor()
{
    return 4000.0;  // so far limited to 8G        
}

float GetSpiRateScaleFactor()
{
    return 64;
}

float  GetSpiRateLimit()
{
    return 500.0;
}

float    GetSpiAccelLimit()
{
    return 8.0;     // so far limited to 8G
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
