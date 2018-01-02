/** ***************************************************************************
 * @file bsp.h Board Support package, configure the Cortex M3
 * @Author whpeng
 * @date   2011-01-21 01:24:06 -0800 (Fri, 21 Jan 2011)
 * @rev 15722
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef CONF_GPIO_H
#define CONF_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f2xx.h"
#include "timer.h"
//#include "iar_stm32f207zg_sk.h"

#define OS_TICKS_PER_SEC 1000
#define SERVOMID		   75

#ifdef FL // disable led
void led_on(Led_TypeDef led);
void led_off(Led_TypeDef led);
void led_toggle(Led_TypeDef led);
#endif


void BSP_init(void);
void RCC_config(void);
void GPIO_config(void);
void NVIC_config(void);
void DelayMs(uint32_t dly_ticks);

#ifdef __cplusplus
}
#endif

#endif /* CONF_GPIO_H */

