/** ***************************************************************************
 * @file extern_port_config.h enumerations relating to the GPS port
 * @author
 * @date   September, 2008
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/

/// constants for port type configuration fields (see DMU specification)
#ifndef PORT_BAUD_CONFIG
    #define PORT_BAUD_CONFIG
    #define BAUD_UNDEFINED -1
    #define BAUD_9600	0
    #define BAUD_19200	1
    #define BAUD_38400	2
    #define BAUD_57600	3
    #define BAUD_4800	4
    #define BAUD_115200	5
    #define BAUD_230400	6
    #define BAUD_460800	7
#endif

/// total number of port baud rates
#ifndef NUM_BAUD_RATES
    #define NUM_BAUD_RATES      8
#endif

// xbowsp_fields.c (set default at startup to 20 Hz, AU pckt at 57.6 kbps)
#define PACKET_RATE_DIVIDER     5
#define PACKET_TYPE             0x4631
#define BAUD_RATE_USER          BAUD_57600  // switches to 38400

// Leave in during compilation to enable N2 packet at 115.2 kbps
//#define PACKET_TYPE             0x4e32      // Nav2 U (N2) = 0x4E32
//#define BAUD_RATE_USER          BAUD_115200 // switches to 38400

