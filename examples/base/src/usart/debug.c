 /** ***************************************************************************
 * @file   debug.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright 2013 by MEMSIC, Inc., all rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  This set of functions formats and sends ASCII messages out USART DEBUG
 *        serial to a serial console commandLine recives serial messages from the
 *        console.
 ******************************************************************************/
#include "debug.h"
#include "debug_usart.h"
#include "utilities.h" // for itoa

// local version
void itoa(int value, char s[], int base);
void itoa_64bit(int64_t value, char s[], int base);

/** ***************************************************************************
 * @name DebugPrintString()
 * @brief sequence through a string one character at a time and send each out
 * the DEBUG serial
 *
 * @param [in] str - the data to send out
 * @retval success = 0
 ******************************************************************************/
void DebugPrintString(const char * str)
{
    while (str && *str) {
        if (*str < 0x7E) { ///< valid printable chars only
            DebugSerialPutChar(*str);
        }
        str++;
    }
}

/** ***************************************************************************
 * @name DebugPrintInt()
 * @brief combine a string and integer then convert the integer to a string and
 * send out the DEBUG serial
 *
 * @param [in] str - string data to send out in front of the integer
 * @param [in] i - integer to convert to a string in decimal format then send out
 * @retval success = 0
 ******************************************************************************/
void DebugPrintInt(const char *s,
                   int        i)
{
    /** 32 bit int: -2147483646
        digits      01234567890 + NULL */
    char numberString[11];

    DebugPrintString(s);
    itoa(i, numberString, 10); ///< base 10
    DebugPrintString(numberString);
}

void DebugPrintLongInt( const char *s, int64_t i )
{
    /* 64 bit int: 9,223,372,036,854,780,000 */
    /* digits      01234567890 + NULL */
    char numberString[20];
    DebugPrintString(s);
    itoa_64bit(i, numberString, 10); // base 10
    DebugPrintString(numberString);
}

/** ***************************************************************************
 * @name DebugPrintHex()
 * @brief combine a string and integer then convert the integer to a hex string
 * and send out the DEBUG serial. Does NOT add a '0x'
 *
 * @param [in] str - string data to send out in front of the integer
 * @param [in] i - integer to convert to a hex base string then send out
 * @retval success = 0
 ******************************************************************************/
void DebugPrintHex(const char *s,
                   int         i)
{
    /** 32 bit int: FFFFFFFF
        digits      01234567 + NULL */
    char numberString[9];

    DebugPrintString(s);
    itoa(i, numberString, 16); ///< base 16
    DebugPrintString(numberString);

}

/** ***************************************************************************
 * @name DebugPrintFloat()
 * @brief combine a string and float then convert the float to a series of
 * digits including the - sign then send out the DEBUG serial.
 *
 * @param [in] str - string data to send out in front of the integer
 * @param [in] f - floating point number to convert to a string then send out
 * @param [in] signbits - flag to indicat a + or - number
 * @retval success = 0
 ******************************************************************************/
void DebugPrintFloat(const char *s,
                     float      f,
                     int        sigDigits)
{
    char numberString[11];
    int i;

    DebugPrintString(s);
    i = (int) f; ///< just get the number to the left of the decimal
    if (i == 0 && f < 0) {
        DebugPrintString("-");
    }
    DebugPrintInt("", i);

    /// now get the number of significant digits to the right
    f = f - i;
    if (f < 0) {
      f = -f;
    }
    if (sigDigits > (sizeof(numberString) -1) ) {
        sigDigits = sizeof(numberString) -1;
    }

    DebugPrintString(".");
    while (sigDigits) {
        f *= 10;
        sigDigits--;
        i = (int) f;
        DebugPrintInt("", i);
        f = f - i;
    }
}

/** ***************************************************************************
 * @name DebugPrintEndLine()
 * @brief send a ascii return and newline out to a serial debug console
 * via the DEBUG serial.
 *
 * @param [in] N/A
 * @retval success = 0
 ******************************************************************************/
void DebugPrintEndline()
{
    DebugPrintString("\r\n");
}

#define MAX_INT32_STRING 11 // 10 digits plus a NULL
#define RADIX_DECIMAL    10
/** ***************************************************************************
 * @name itoa()
 * @brief local version of integer to ascii
 *
 * @param [in] value - integer to convert
 * @param [in] sp - output buffer
 * @param [in] radix - number base default base 10
 * @retval N/A
 ******************************************************************************/
void itoa(int  value,
          char *sp,
          int  radix)
{
    char     tmp[MAX_INT32_STRING]; // [11]
    char     *tp = tmp;
    int      i;
    unsigned v;
    int      sign;

    sign = (radix == RADIX_DECIMAL && value < 0);
    if (sign) {
        v = -value;
    } else {
        v = (unsigned)value;
    }

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

    if (radix != RADIX_DECIMAL) { ///< zero fill non decimal values
        while (tp < &(tmp[4])) {
          *tp++ = '0';
        }
    }

    if (sign) { *sp++ = '-'; }
    /// reverse and put in output buffer
    while (tp > tmp) {
        tp--;
        *sp = *tp;
        sp++;
    }
    *sp = '\0';
}

#define MAX_INT64_STRING 20 // 10 digits plus a NULL

void itoa_64bit( int64_t value, char *sp, int radix )
{
    char tmp[MAX_INT64_STRING];
    char *tp = tmp;
    int i;
    uint64_t v;
    int sign;

    sign = ( (radix == RADIX_DECIMAL) && (value < 0) );
    if (sign) {
        v = (uint64_t)( -value );
    } else {
        v = (uint64_t)value;
    }

    while( v || tp == tmp )
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
