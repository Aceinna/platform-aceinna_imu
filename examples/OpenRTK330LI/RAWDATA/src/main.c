
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

#define __MAIN
#include "main.h"
#include "configureGPIO.h"
#include "bsp.h"
#include "uart.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"
#include "user_config.h"
#include "timer.h"


osThreadId IMU_DATA_ACQ_TASK;
osThreadId GNSS_RTK_TASK;
osThreadId GNSS_DATA_ACQ_TASK;
osThreadId ETHERNET_TASK;

osSemaphoreDef(IMU_DATA_ACQ_SEM);
osSemaphoreDef(RTK_START_SEM);
osSemaphoreDef(RTK_FINISH_SEM);


osSemaphoreId g_sem_imu_data_acq;
osSemaphoreId g_sem_rtk_start;
osSemaphoreId g_sem_rtk_finish;

/** ***************************************************************************
 * @name CreateTasks()
 * @brief CreateTasks
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void CreateTasks(void)
{
    osThreadId iD;

    /// Create RTOS tasks:
    // dacq task
    g_sem_imu_data_acq = osSemaphoreCreate(osSemaphore(IMU_DATA_ACQ_SEM), 1);
    g_sem_rtk_start = osSemaphoreCreate(osSemaphore(RTK_START_SEM), 1);
    g_sem_rtk_finish = osSemaphoreCreate(osSemaphore(RTK_FINISH_SEM), 1);

    osThreadDef(IMU_DATA_ACQ_TASK, TaskDataAcquisition, osPriorityRealtime, 0, TASK_IMU_DATA_ACQ_STACK);
    iD = osThreadCreate(osThread(IMU_DATA_ACQ_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ; 
    }

    osThreadDef(GNSS_DATA_ACQ_TASK, GnssDataAcqTask, osPriorityBelowNormal, 0, TASK_GNSS_DATA_ACQ_STACK);
    iD = osThreadCreate(osThread(GNSS_DATA_ACQ_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ;
    }

    osThreadDef(GNSS_RTK_TASK, RTKTask, osPriorityLow, 0, TASK_GNSS_RTK_STACK);
    iD = osThreadCreate(osThread(GNSS_RTK_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ;
    }

    osThreadDef(ETHERNET_TASK, EthTask, osPriorityNormal, 0, TASK_ETHERNET_STACK);
    iD = osThreadCreate(osThread(ETHERNET_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ;
    }

}


/** ***************************************************************************
 * @name main()
 * @brief mian
 * @param N/A
 * @retval N/A
 ******************************************************************************/
int main(void)
{
    // Initialize processor and board-related signals  
    BoardInit();
    MX_DMA_Init();

    uart_driver_install(UART_DEBUG,&uart_debug_rx_fifo,&huart_debug,460800);
    uart_driver_install(UART_USER,&uart_user_rx_fifo,&huart_user,460800);
    uart_driver_install(UART_GPS,&uart_gps_rx_fifo,&huart_gps,460800);
    uart_driver_install(UART_BT,&uart_bt_rx_fifo,&huart_bt,460800);

    ResetForEnterBootMode();  // normal or iap mode

    InitFactoryCalibration();
    ApplyFactoryConfiguration();
    userInitConfigureUnit();

    CreateTasks();
    MX_TIM_SENSOR_Init();

    /* Infinite loop */
    while ( 1 ) 
    {

        osKernelStart();

        // We should never get here as control is now taken by the scheduler
        for (;;)
            ;

    }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

void  vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    while(1);
}

void  vApplicationMallocFailedHook( void )
{
    while(1);
}


