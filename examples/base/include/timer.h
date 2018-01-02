/** ***************************************************************************
 * @file timer.h system timer configure an daccess functions
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
#ifndef __TIMER_H
#define __TIMER_H
#include <stdint.h>

#include "stm32f2xx.h"

typedef struct
{
  uint32_t seconds;
  uint32_t milliseconds;
} sys_time_t;

typedef uint32_t tTime;

#define TIME_MAX UINT32_MAX

void DelayMs( tTime delay);
tTime TimeNow();
tTime TimePassed(tTime since);
void InitSystemTimer();

#endif
