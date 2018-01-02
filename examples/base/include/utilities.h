/** ***************************************************************************
 * @file   utilities.h
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * local version of string functions
 ******************************************************************************/

#ifndef UTILITIES_H
#define UTILITIES_H
#include <stdint.h>

extern char* strtok_r1(char *str, char const token, char **cursor);
extern void strrep(char *str, const char find, const char rep);
extern int strcmpi(char const *a, char const *b);
extern void itoa(int value, char *sp, int radix);
uint16_t byteSwap16(uint16_t b);
uint32_t byteSwap32( uint32_t val );

#endif