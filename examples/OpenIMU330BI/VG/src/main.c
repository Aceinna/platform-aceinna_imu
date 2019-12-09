/** ***************************************************************************
 * @file main.c
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

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "configurationAPI.h"
#include "calibrationAPI.h"
#include "hwAPI.h"
#include "osapi.h"
#include "sensorsAPI.h"
#include "UserCommunicationSPI.h"
#include "taskDataAcquisition.h"
#include "UserConfiguration.h"
#include "platformAPI.h"
#include "debug.h"


int main(void)
{
    
    int res, rate; 
    
    HW_Init();
    platformInitBITStatus();    
    InitFactoryCalibration();
    ApplyFactoryConfiguration();
    ApplyUserConfiguration();

    res = InitSensors();
    
    if(!res){
        platformSetSensorFaultStatus();    
    }

    // parameter is timer resolution in uS
    StartReferenceTimer(1);

    if(ExtSyncEnabled()){
        res = platformActivateExternalSync(ExtSyncFrequency());   // enable external 1KHZ sync
    }
    
    if(fSPI){
        InitUserCommunicationSPI();
    }else{
        rate = configGetBaudRate();
        //  User serial port defaulted to 0 (USER_SERIAL_PORT) but can be redefined using next
        //  function if needed. Debug port in this case reassigned to user port if debug enabled
        //  platformSetUserSerialPort(DEBUG_SERIAL_PORT);
        platformInitUserSerialPort(rate);
    }

#ifdef DEBUG_ENABLED
    InitDebugSerialCommunication(115200);
#endif

    TaskDataAcquisition();
    
    // should never return here
    while(1);
}

