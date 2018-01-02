/** ***************************************************************************
 * @file   accerometer.h
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * generic accelerometer interface, it should be implemented
 * by whichever accelerometer is in use
 *****************************************************************************/
#ifndef COMPILER_FLAGS_H
#define COMPILER_FLAGS_H

// FIXME: Set to one if the 25 MHz oscillator is used.  Remove once the board is
//        fixed.
#ifdef MTLT_BOARD
#define  TWENTY_FIVE_MHZ_OSCILLATOR  1
#endif

// Set to one during compilation to enable N1 packet at 57.6 kbps (25 Hz ODR)
//   if configuration is invalid.
//#define  N1_DEFAULT_PCKT  0


//#define USE_TIM5_IN_TASK_DAQ 1


//#define  ORIG_UART_OUTPUT_TIMING  1
//#define  SEMAPHORE_BASED_UART_OUTPUT_TIMING_NO_TIMEOUT  0

#endif /* COMPILER_FLAGS_H */
