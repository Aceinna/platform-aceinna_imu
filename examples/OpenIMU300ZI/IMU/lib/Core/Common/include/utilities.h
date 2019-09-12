/** ***************************************************************************
 * @file   utilities.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * local version of string functions
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

#ifndef UTILITIES_H
#define UTILITIES_H
#include <stdint.h>

extern char* strtok_r1(char *str, char const token, char **cursor);
extern void strrep(char *str, const char find, const char rep);
extern int strcmpi(char const *a, char const *b);
//extern void itoa(int value, char *sp, int radix);
uint16_t byteSwap16(uint16_t b);
uint32_t byteSwap32( uint32_t val );

#endif
