/** ***************************************************************************
 * @file   main.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  main is the center of the system universe, it all starts here. And never ends.
 * entry point for system (pins, clocks, interrupts), data and task initialization.
 * contains the main processing loop. - this is a standard implementation
 * which has mainly os functionality in the main loop
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
#define __MAIN

#include <stddef.h>

#include "boardAPI.h"
#include "magAPI.h"
#include "platformAPI.h"
#include "userAPI.h"
#include "debug.h"

#include "taskDataAcquisition.h"
#include "taskCanCommunicationJ1939.h"

#include "osapi.h"
#include "osresources.h"
#include "canAPI.h"

char buildInfo[] = {__DATE__"," __TIME__};

/** ***************************************************************************
 * @name getBuildInfo - provides the pointer to the build date and time string
 * @brief  
 *
 * @param [in] N/A
 * @param [out]  pointer to build info string
 * @retval N/A
 ******************************************************************************/
char *getBuildInfo()
{
    return buildInfo;
}

/** ***************************************************************************
 * @name initDebugInterface Debug serial interface initialization
 * @brief  Initializes debug serial console messaging support
 *
 * @param [in] N/A
 * @param [out] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/
void DebugInterfaceInit(void)
{
    char status[100];

    BOOL res  = platformAssignPortTypeToSerialChannel(DEBUG_SERIAL_PORT, UART_CHANNEL_0);   // assign debug output to user serial channel

    if(!res){
        // some error 
        return;
    }
#ifdef GPS_OVER_CAN
    res  = platformAssignPortTypeToSerialChannel(USER_SERIAL_PORT, UART_CHANNEL_0);   // assign debug output to user serial channel
    if(!res){
        // some error 
        return;
    }
#endif

    // Initialize the debug serial port
    InitDebugSerialCommunication(38400);

    BoardGetResetStatus(status, sizeof(status));

}

void CreateTasks(void)
{
    osThreadId iD;
    
    /// Create RTOS tasks:
    // dacq task
    osThreadDef(DACQ_TASK, TaskDataAcquisition, osPriorityAboveNormal, 0, 2048);
    iD = osThreadCreate(osThread(DACQ_TASK), NULL);
    if(iD == NULL){
        while(1);
    }

    //user communication task
    osThreadDef(CAN_COMM_TASK, TaskCANCommunicationJ1939, osPriorityNormal, 0, 2048);
    iD = osThreadCreate(osThread(CAN_COMM_TASK), NULL);
    if(iD == NULL){
        while(1);
    }

#ifdef GPS
    // GPS task
    osThreadDef(GPS_TASK,  TaskGps, osPriorityBelowNormal, 0, 1024);
    iD = osThreadCreate(osThread(GPS_TASK), NULL);
    if(iD == NULL){
        while(1);
    }
    
    // WMM task
    osThreadDef(WMM_TASK,  TaskWorldMagneticModel, osPriorityLow, 0, 512);
    iD = osThreadCreate(osThread(WMM_TASK), NULL);
    if(iD == NULL){
        while(1);
    }

#endif


    gyroReadySem      = osSemaphoreCreate(osSemaphore(GYRO_READY_SEM), 1);
    accelReadySem     = osSemaphoreCreate(osSemaphore(ACCEL_READY_SEM), 1);
    magReadySem       = osSemaphoreCreate(osSemaphore(MAG_READY_SEM), 1);
    tempReadySem      = osSemaphoreCreate(osSemaphore(TEMP_READY_SEM), 1);
    canDataSem        = osSemaphoreCreate(osSemaphore(CAN_DATA_SEM), 1);
    dataAcqSem        = osSemaphoreCreate(osSemaphore(DATA_ACQ_SEM), 1);

}

/** ***************************************************************************
 * @name main() The DMU380 firmware initialization and entry point
 * @brief creates tasks for data collection SPI1 and user communications SPI3
 *        or USART serial stream (Nav-View) debug serial console messaging
 *
 * @param [in] N/A
 * @param [out] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/
int main(void)
{
#ifdef GPS
    BOOL res;
#endif

    // Initialize processor and board-related signals  
    BoardInit();

    platformSetUnitCommunicationType(CAN_BUS);

 #ifdef GPS
    platformUnassignSerialChannels();

    #ifndef GPS_OVER_CAN
    // user serial port will be used for GPS
    res = platformAssignPortTypeToSerialChannel(GPS_SERIAL_PORT, UART_CHANNEL_0);   
    while(!res){};  // check if valid
    #else
       DebugInterfaceInit();
    #endif
#else
    DebugInterfaceInit();
#endif

    // Apply factory configuration
    platformInitConfigureUnit(); 

    // Apply user-chosen configuration
    userInitConfigureUnit(); 

    // Initialize OS and create required tasks
    CreateTasks();
 
    // Start Running the tasks...
    // Start scheduler
    osKernelStart();

    // We should never get here as control is now taken by the scheduler
    for (;;);

}


void vApplicationStackOverflowHook()
{
    while(1);
}
void vApplicationMallocFailedHook()
{
     while(1);
}

