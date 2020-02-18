/** ****************************************************************************
 * @file   debug.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief
 * This is a compile-time prioritized debug message driver. It allows sending
 * strings (and numbers) through a serial USART port. It only prints variables
 * at the end of the string:
 *    DEBUG_STRING("This takes a string");
 *    DEBUG_INT("This prints a string then an integer: ", integer_variable);
 *    DEBUG_HEX("String then integer in hex at the end: ", integer_variable);
 *    DEBUG_FLOAT("String then a float: ", float_variable, significant_digits);
 *    DEBUG_TIMESTAMP("Prints a 64 but integer: ", timestamp_variable);
 *    DEBUG_ENDLINE(); // outputs "\r\n"
 *
 * It also lets you leave debug print statements for future debugging
 * without incurring a code-space cost. Each logging function starts with
 * INFO_, DEBUG_, or ERROR_ prefix. By default, for DEBUG builds,
 * all DEBUG_* (e.g., DEBUG_STRING)  send their data to the debug uart port.
 * Also all ERROR_* logs are sent to the debug port (that may remain true
 * for production builds if we have a debug uart).
 *
 * If you want to see the INFO_* logs, you'll have to set the logging level
 * in your file at compile time, before including this file.
 *     #define LOGGING_LEVEL LEVEL_INFO
 *     #include "debug.h"
 * And if you are getting too much data, you can also set your logging level
 * higher to quiet a module:
 *     #define LOGGING_LEVEL LEVEL_ERROR
 *     #include "debug.h"
 *****************************************************************************/
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


#ifndef _IMU_DEBUG_H
#define _IMU_DEBUG_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

extern void InitSerialCommunication(void);

#define LEVEL_STREAM -1 // only run stream when set - no debug or error
#define LEVEL_NONE    0
#define LEVEL_ERROR   1
#define LEVEL_DEBUG   2
#define LEVEL_INFO    3

#ifdef GPS_ON_DEBUG_PORT
#define LOGGING_LEVEL LEVEL_NONE    // cannot output log messages 
#endif

#ifndef LOGGING_LEVEL
#define LOGGING_LEVEL LEVEL_DEBUG
#endif

// made the level negative to turn off all other messaging when using this port for streaming
#if LOGGING_LEVEL == LEVEL_STREAM
#define STREAM_STRING(s)       DebugPrintString((s));
#define STREAM_INT(s, i)       DebugPrintInt((s),(i));
#define STREAM_UINT(s, i)      DebugPrintUInt((s),(i));
#define STREAM_HEX(s, i)       DebugPrintHex((s),(i));
#define STREAM_TIMESTAMP(s, t) DebugPrintHex((s),(t >> 32)); DebugPrintHex(NULL, (t)&0xFFFFFFFF);
#define STREAM_FLOAT(s, i, d)  DebugPrintFloat((s),(i), (d));
#define STREAM_ENDLINE()       DebugPrintEndline();
#else // using this port to stream data
#define STREAM_STRING(s)
#define STREAM_INT(s, i)
#define STREAM_HEX(s, i)
#define STREAM_TIMESTAMP(s, t)
#define STREAM_FLOAT(s, i, d)
#define STREAM_ENDLINE()
#endif


#if LOGGING_LEVEL >= LEVEL_DEBUG
#define DEBUG_STRING(s)       DebugPrintString((s));
#define DEBUG_INT(s, i)       DebugPrintInt((s),(i));
#define DEBUG_UINT(s, i)      DebugPrintUInt((s),(i));
#define DEBUG_LONGINT(s, i)   DebugPrintLongInt((s),(int64_t)(i));
#define DEBUG_HEX(s, i)       DebugPrintHex((s),(i));
#define DEBUG_TIMESTAMP(s, t) DebugPrintHex((s),(t >> 32)); DebugPrintHex(NULL, (t)&0xFFFFFFFF);
#define DEBUG_FLOAT(s, i, d)  DebugPrintFloat((s),(i), (d));
#define DEBUG_ENDLINE()       DebugPrintEndline();
#else
#define DEBUG_STRING(s)
#define DEBUG_INT(s, i)
#define DEBUG_LONGINT(s, i)
#define DEBUG_HEX(s, i)
#define DEBUG_TIMESTAMP(s, t)
#define DEBUG_FLOAT(s, i, d)
#define DEBUG_ENDLINE()
#endif

#if LOGGING_LEVEL >= LEVEL_INFO
#define INFO_STRING(s)       DebugPrintString((s));
#define INFO_INT(s, i)       DebugPrintInt((s),(i));
#define INFO_UNT(s, i)       DebugPrintUInt((s),(i));
#define INFO_HEX(s, i)       DebugPrintHex((s),(i));
#define INFO_TIMESTAMP(s, t) DebugPrintHex((s),(t >> 32)); DebugPrintHex(NULL, (t)&0xFFFFFFFF);
#define INFO_FLOAT(s, i, d)  DebugPrintFloat((s),(i), (d));
#define INFO_ENDLINE()       DebugPrintEndline();
#else
#define INFO_STRING(s)
#define INFO_INT(s, i)
#define INFO_HEX(s, i)
#define INFO_TIMESTAMP(s, t)
#define INFO_FLOAT(s, i, d)
#define INFO_ENDLINE()
#endif

#if LOGGING_LEVEL >= LEVEL_ERROR
#define ERROR_STRING(s)       DebugPrintString((s));
#define ERROR_INT(s, i)       DebugPrintInt((s),(i));
#define ERROR_UINT(s, i)      DebugPrintUInt((s),(i));
#define ERROR_HEX(s, i)       DebugPrintHex((s),(i));
#define ERROR_TIMESTAMP(s, t) DebugPrintHex((s),(t >> 32)); DebugPrintHex(NULL, (t)&0xFFFFFFFF);
#define ERROR_FLOAT(s, i, d)  DebugPrintFloat((s),(i), (d));
#define ERROR_ENDLINE()       DebugPrintEndline();
#else
#define ERROR_STRING(s)
#define ERROR_INT(s, i)
#define ERROR_HEX(s, i)
#define ERROR_TIMESTAMP(s, t)
#define ERROR_FLOAT(s, i, d)
#define ERROR_ENDLINE()
#endif

extern void           InitDebugSerialCommunication( uint32_t baudRate );
extern unsigned char  DebugPutChar (unsigned char c);
extern int            DebugReadLine(uint8_t *buf, uint32_t *index, uint32_t len);
extern void DebugPrintString(const char * str);
extern void DebugPrintInt(const char *str, int i);
extern void DebugPrintUInt(const char *str, uint32_t i);
extern void DebugPrintLongInt( const char *s, int64_t i );
extern void DebugPrintHex(const char *str, int i);
extern void DebugPrintFloat(const char *str, float f, int sigDigits);
extern void DebugPrintEndline();
extern int            IsDebugSerialIdle();

int  tprintf(char *format, ...);



#endif /* DEBUG_H */