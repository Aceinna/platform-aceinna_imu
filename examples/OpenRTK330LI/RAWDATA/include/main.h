/*****************************************************************************
 * @file main.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MAIN_H
#define _MAIN_H

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#include <stdint.h>
#include <stdio.h>

#include "cmsis_os.h"
#include "portmacro.h"
#ifdef __cplusplus
extern "C"
{
#endif


void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

void TaskDataAcquisition(void const *argument);
void GnssDataAcqTask(void const *argument);
void EthTask(void const *argument);
void RTKTask(void const *argument);

extern osSemaphoreId g_sem_imu_data_acq;
extern osSemaphoreId g_sem_rtk_start;
extern osSemaphoreId g_sem_rtk_finish;


#ifdef __cplusplus
}
#endif

#define CCMRAM __attribute__((section(".ccmram")))

#endif /* _MAIN_H */

