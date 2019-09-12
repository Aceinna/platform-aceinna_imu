/** ***************************************************************************
 * @file   utilities.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * string functions
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


#include "utilities.h"

/** ****************************************************************************
 * @name: tolower
 * @brief reimplementation of library function:
 *        change uppercase letters to lowercase
 *  (This is just a helper function for strcmpi.)
 * @param [in] c - character of interest
 * @retval the character of interest (lowercase if it was upper)
 ******************************************************************************/
//char tolower(char c) {
char tlwr(char c) {
  if ('A' <= c && c <= 'Z') { 
    return (char)((c-'A')+'a');
  }
  return c;
}

/** ****************************************************************************
 * @name: strcmpi
 * @brief reimplementation of library function:
 *        compare two string without regard for case
 *
 * @param [in] two strings to be compared (a and b)
 * @retval strcmp returns 0 when the strings are equal, a negative
 *   integer when s1 is less than s2, or a positive integer if s1 is greater
 *   than s2, according to the lexicographical order.
 ******************************************************************************/
int strcmpi(char const *a,
            char const *b) {

  while ((*a) && (*b) && tlwr(*a) == tlwr(*b)) {
    a++;
    b++;
  }
  if (*a > *b) {
    return 1;
  } else if (*a < *b) {
    return -1;
  } else { //(*a == *b)
    return 0;
  }
}

/** ****************************************************************************
 * @name: strrep
 * @brief replace a character inside a string, for example to
 *   replace newline characters with string termination characters.
 *
 * @param [in] str - string to be modified
 * @param [in] find -  character to be replaced
 * @param [in] rep - replacement character
 * @retval None.
*******************************************************************************/
void strrep(char       *str,
            const char find,
            const char rep) {
	while (*str) {
		if (find == *str) {
			*str = rep;
		}
		str++;
	}
}

/** ****************************************************************************
 * @name: strtok_r
 * @brief reimplementation of library function:
 *        parses a string into a sequence of tokens
 *  The strtok_r() function is a reentrant version of strtok().
 *  It gets the next token from string s1, where tokens are strings separated
 *  by characters from s2. To get the first token from s1, strtok_r() is
 *  called with s1 as its first parameter. Remaining tokens from s1 are
 *  obtained by calling strtok_r() with a null pointer for the first
 *  parameter. The string of delimiters, s2, can differ from call to call.
 *
 *  @param [in] str - string to be searched
 *  @param [in] token - to use for separation
 *  @param [in] cursor - points to the last found token
 *  @retval returns the next token to be used.
*****************************************************************************/
char* strtok_r1(char       *str,
               char const token,
               char       **cursor) {
	char *first;
	char *last;

	/// On the first call, we initialize from str
	/// On subsequent calls, str can be NULL
	if (str) {
		first = str;
	} else {
		first = *cursor;
		if (!first) {
		  return 0;
		}
	}
	/// Skip past any initial tokens
	while (*first && (*first == token)) {
		first++;
	}
	/// If string ends before first token is found, abort
	if (0 == *first) {
		return 0;
	}

	/// Now start moving last
	last = first + 1;
	while (*last && (*last != token)) {
		last++;
	}
	/// Found either end-of-token or end of string
	if (*last) {
		/// End of token
		*cursor = last + 1;
		*last = 0;
	} else {
		*cursor = 0;
	}
	return first;
}


/*****************************************************************************
 ** Function name: itoa
 ** Descriptions: reimplementation of library function:
 **     Converts an integer value to a null-terminated string using the
 **     specified base and stores the result in the array given by str parameter.
 ** parameters:   value to be converted
 **               string to put it in
 **               base of the numer
 ** Returned value: none.
*****************************************************************************/
#define MAX_INT32_STRING 11 // 10 digits plus a NULL
#define RADIX_DECIMAL    10

#ifdef FL
void itoa(int value, char *sp, int radix)
{
    char tmp[MAX_INT32_STRING];
    char *tp = tmp;
    int i;
    unsigned v;
    int sign;

    sign = (radix == RADIX_DECIMAL && value < 0);
    if (sign)  { v = -value; }
    else       { v = (unsigned)value; }

    while (v || tp == tmp)
    {
        i = v % radix;
        v /= radix;
        if (i < RADIX_DECIMAL) {
          *tp++ = i+'0';
        } else {
          *tp++ = i + 'A' - RADIX_DECIMAL;
        }
    }

    if (radix != RADIX_DECIMAL) { // zero fill non decimal values
        while (tp < &(tmp[4])) {
          *tp++ = '0';
        }
    }

    if (sign) { *sp++ = '-'; }
    // reverse and put in output buffer
    while (tp > tmp) {
        tp--;
        *sp = *tp;
        sp++;
    }
    *sp = '\0';

}
#endif

/** ****************************************************************************
 * @name byteSwap16
 * @param [in] b - short word to swap endieness
 * @retval byte swappeed word
 ******************************************************************************/
uint16_t byteSwap16(uint16_t b)
{ return (b << 8) | (b >> 8); }

/** ****************************************************************************
 * @name byteSwap32
 * @param [in] b - short word to swap endieness
 * @retval byte swappeed word
 ******************************************************************************/
uint32_t byteSwap32( uint32_t val )
{
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | (val >> 16);
}

