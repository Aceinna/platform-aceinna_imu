/** ***************************************************************************
 * @file   lowpass_filter.h
 * @Author
 * @date   March, 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * generic accelerometer interface, it should be implemented
 * by whichever accelerometer is in use
 *****************************************************************************/
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#define BWF_LOWPASS_3RD_2        0
#define BWF_LOWPASS_3RD_5        1
#define BWF_LOWPASS_3RD_10       2
#define BWF_LOWPASS_3RD_20       3
#define BWF_LOWPASS_3RD_25       4
#define BWF_LOWPASS_3RD_40       5
#define BWF_LOWPASS_3RD_50       6
#define BWF_LOWPASS_3RD_INF      7

#define BWF_LOWPASS_DATA_RATE_400       0
#define BWF_LOWPASS_DATA_RATE_800       1

#define  LPF_100HZ  100
#define  LPF_50HZ   50
#define  LPF_40HZ   40
#define  LPF_25HZ   25
#define  LPF_20HZ   20
#define  LPF_10HZ   10
#define  LPF_5HZ    5
#define  LPF_UNFIL  0



extern uint8_t _rateFilt_3rdOrderBWF_LowPass_Axis(uint8_t, int16_t, int32_t *, uint8_t, uint8_t);
extern uint8_t _accelFilt_3rdOrderBWF_LowPass_Axis(uint8_t, int16_t, int32_t *, uint8_t, uint8_t);

extern uint8_t _rateFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(uint8_t, uint8_t, int16_t, int32_t *, uint8_t, uint8_t);
extern uint8_t _accelFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(uint8_t, uint8_t, int16_t, int32_t *, uint8_t, uint8_t);

extern uint8_t _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(uint8_t chip, uint8_t, int16_t, int32_t *, uint8_t, uint8_t);
extern uint8_t _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(uint8_t chip, uint8_t, int16_t, int32_t *, uint8_t, uint8_t);

#endif

