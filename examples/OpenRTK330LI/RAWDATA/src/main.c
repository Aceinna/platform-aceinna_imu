
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
*******************************************************************************
* 16/10/2019  |                                             | Daich
* Description: use uart_driver_install to init uart
* 16/10/2019  |                                             | Daich
* Description: remove some warning
* 24/10/2019  |                                             | Neil
* Description: Modify task priority, represented by enumerated variables
               GPS has a higher priority than RTD 
* 25/10/2019  |                                             | Neil
* Description: Modify task priority, represented by enumerated variables
               GPS has a lower priority than RTD ;
               Because the RTD task does not use caching, it prevents the data 
               from being changed during the calculation;
* 16/12/2019  |                                             | Daich
* Description: create usat idle interrupt sem
* 06/01/2020  |                                             | Daich
* Description: create bt_uart_sem sem
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#define __MAIN
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "osresources.h"
#include "gpsAPI.h"
#include "userAPI.h"
#include "configureGPIO.h"
#include "taskRTK.h"
#include "boardDefinition.h"
#include "timer.h"
#include "bsp.h"
#include "uart.h"
#include "main.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "taskDataAcquisition.h"
#include "osresources.h"
#include "taskEth.h"
#include "UserConfiguration.h"
#include "shell.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


osMutexDef(BT_MUTEX);
osMutexId bt_mutex;

osThreadId DACQ_TASK;
osThreadId RTK_TASK;
osThreadId RTD_TASK;
osThreadId GPS_TASK;
osThreadId ETH_TASK;
osThreadId CONSOLE_TASK;


void mutex_init()
{
#if SHELL_ENABLE == true
    bt_mutex = osMutexCreate(osMutex(BT_MUTEX));
    osMutexRelease(bt_mutex);
#endif

}

void CreateTasks(void)
{
    osThreadId iD;

    /// Create RTOS tasks:
    // dacq task
    dataAcqSem = osSemaphoreCreate(osSemaphore(DATA_ACQ_SEM), 1);
    RTKStartSem = osSemaphoreCreate(osSemaphore(RTK_START_SEM), 1);
    RTKFinishSem = osSemaphoreCreate(osSemaphore(RTK_FINISH_SEM), 1);

    osThreadDef(DACQ_TASK, TaskDataAcquisition, osPriorityRealtime, 0, TASKIMU_STACK);
    iD = osThreadCreate(osThread(DACQ_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ; 
    }

    osThreadDef(GPS_TASK, GnssDataAcqTask, osPriorityBelowNormal, 0, TASKGPS_STACK);
    iD = osThreadCreate(osThread(GPS_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ;
    }

    osThreadDef(RTK_TASK, RTKTask, osPriorityLow, 0, TASKRTK_STACK);
    iD = osThreadCreate(osThread(RTK_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ;
    }

    osThreadDef(ETH_TASK, EthTask, osPriorityNormal, 0, TASKETH_STACK);
    iD = osThreadCreate(osThread(ETH_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ;
    }
#if SHELL_ENABLE == true
#if SHELL_TASK == true
    osThreadDef(CONSOLE_TASK, ConsoleTask, osPriorityLow, 0, TASKCONSOLE_STACK);
    iD = osThreadCreate(osThread(CONSOLE_TASK), NULL);
    if (iD == NULL)
    {
        while (1)
            ;
    }    
#endif
#endif
}

void  vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    while(1);
}
void  vApplicationMallocFailedHook( void )
{
    while(1);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */

int main(void)
{
    // Initialize processor and board-related signals  
    BoardInit();
    initGPSDataStruct();
    mutex_init();
    MX_DMA_Init();
    uart_driver_install(UART_DEBUG,&uart_debug_rx_fifo,&huart_debug,460800);
    uart_driver_install(UART_USER,&uart_user_rx_fifo,&huart_user,460800);
    uart_driver_install(UART_GPS,&uart_gps_rx_fifo,&huart_gps,460800);
    uart_driver_install(UART_BT,&uart_bt_rx_fifo,&huart_bt,460800);

    ResetForEnterBootMode();  // normal or iap mode

    InitFactoryCalibration();
    ApplyFactoryConfiguration();
    userInitConfigureUnit();

    //insinitsystemfromcfg();
    CreateTasks();
    MX_TIM_SENSOR_Init();

    printf("openrtk init complete\r\n");

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
        tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

