/** ***************************************************************************
 * @file driverGPS.c serial interface and buffer functions
 * @author
 * @date   September, 2008
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
#include <stdint.h>

#include "port_def.h"
#include "driverGPS.h"
#include "extern_port_config.h"
#include "uart.h"
#include "comm_buffers.h"
#include "debug.h"
#include "debug_usart.h"  // for UseExtUSARTGps()

port_struct* gGpsUartPtr = 0;
port_struct onePpsUart;
unsigned int gUartChannel;

// extern_port.c
extern void GetGpsExternPortAndChannel(unsigned int* uartChannel,
                                       port_struct** gGpsUartPtr);

/** ****************************************************************************
 * @name baudEnumToBaudRate LOCAL convert enumeration to value
 * @param [in] baudRate - enumeration to translate extern_port_config.h
 * @retval actual baud rate value
 ******************************************************************************/
int32_t baudEnumToBaudRate(int baudEnum)
{   // -1 (invalid) can be sent in. Passed through to gGpsDataPtr->GPSConfigureOK
    uint32_t baudRate = baudEnum;

    switch (baudEnum) {
        case BAUD_9600: // 0
            baudRate =    9600;
            break;
        case BAUD_19200: // 1
            baudRate =   19200;
            break;
        case BAUD_38400: // 2
            baudRate =	 38400;
            break;
        case BAUD_57600: // 3
            baudRate =	 57600;
            break;
        case BAUD_4800:  // 4
            baudRate =	  4800;
            break;
        case BAUD_115200: // 5
            baudRate =	115200;
            break;
        case BAUD_230400: // 6
            baudRate =  230400;
            break;
        case BAUD_460800: // 7
            baudRate =	460800;
            break;
    }

    return baudRate;
}

/** ****************************************************************************
 * @name: initGpsUart set up baudrate and GPS UART
 * @param [in] baud - enumeration to translate 0-7, neg autobaud, otherwise spi
 * @retval N/A
 ******************************************************************************/
void initGpsUart(int32_t baud) {
    baud = baudEnumToBaudRate(baud);
    if (baud > 0) {
        GetGpsExternPortAndChannel(&gUartChannel, // channel 1
                                   &gGpsUartPtr); // port 1
        gGpsUartPtr->hw.baud = baud;
        uart_init(gUartChannel, // low levl (pin level) init
                  &(gGpsUartPtr->hw));
    }
}

// not used
/** ****************************************************************************
 * @name: initOnePpsUart set up baudrate for INTERNAL GPS UART
 * @param [in]
 * @retval N/A
 ******************************************************************************/
void initOnePpsUart( void ) {
    port_struct* onePpsUartPtr = &onePpsUart;

    GetGpsExternPortAndChannel(&gUartChannel,   // channel 1
                               &onePpsUartPtr); // port 1
    onePpsUart.hw.baud = 4800;
    uart_init(gUartChannel, // low levl (pin level) init
              &(onePpsUartPtr->hw));
}

/** ****************************************************************************
 * @name: setExternalPortPtr external GPS USART so pass in port pointer to the
 *        cicular buffer
 * @param [in] gGpsPortPtr - debug USART port struct pointer (circular buffer)
 * @retval N/A
 ******************************************************************************/
void setExternalPortPtr(port_struct* gGpsPortPtr) {
        gGpsUartPtr = gGpsPortPtr;
}

/** ****************************************************************************
 * @name: gpsBytesAvailable bytes in Gps circ buffer API only used in
 *        driverGPSAllEntrance
 * @param N/A
 * @retval number of bytes in Gps circ buffer
 ******************************************************************************/
int gpsBytesAvailable()
{
    if ( gGpsUartPtr ) {
        return (COM_buf_bytes_available(&(gGpsUartPtr->rec_buf)));
    }
   return 0;
}

/** ****************************************************************************
 * @name: flushGPSRecBuf empty GPS interface's software buffer.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void flushGPSRecBuf(void)
{
	uint16_t numToRemove;

    numToRemove = COM_buf_bytes_available(&(gGpsUartPtr->rec_buf));
    COM_buf_delete(&(gGpsUartPtr->rec_buf),
                   numToRemove);
}

/** ****************************************************************************
 * @name isGpsTxEmpty check if transmit buffer is empty...
 * @param N/A
 * @retval N/A
 ******************************************************************************/
BOOL isGpsTxEmpty(void)
{
    return (0 == bytes_remaining(gUartChannel, gGpsUartPtr)); // uart.c
}

/** ****************************************************************************
 * @name delBytesGpsBuf will pop bytes off in the receive FIFO.
 * @param [in] numToPop is the number of bytes to pop off
 * @retval the number of bytes remaining in the FIFO.
 ******************************************************************************/
uint16_t delBytesGpsBuf(uint16_t numToPop)
{
	int16_t numInBuffer;

    numInBuffer = COM_buf_bytes_available(&(gGpsUartPtr->rec_buf));
    if (numInBuffer < numToPop) {
        numToPop = numInBuffer;
    }
    COM_buf_delete(&(gGpsUartPtr->rec_buf),
                   numToPop);
    return ( numInBuffer - numToPop ); ///< unscanned bytes
}

/** ****************************************************************************
 * @name peekWordGpsBuf will return a word without popping the FIFO.
 * @brief No error checking on the range of index is performed.
 * @param [in] index represents the byte offset into the FIFO to start looking at.
 * @retval  the 16-bit word in the FIFO starting at index offset from end.
 ******************************************************************************/
uint16_t peekWordGpsBuf(uint16_t index)
{
	uint16_t word;

    word = peekByteGpsBuf(index) << 8 | peekByteGpsBuf(index + 1);
	return word;
}

/** ****************************************************************************
 * @name peekByteGpsBuf peekByteSCIA will return a byte without popping
 *       the FIFO.
 * @brief No error checking on the range of index is performed.
 * @param [in] index represents the byte offset into the FIFO to look at.
 * @retval the byte in the FIFO pointed to by index.
 ******************************************************************************/
uint8_t peekByteGpsBuf(uint16_t index)
{
    uint8_t output = 0;

    COM_buf_copy (&(gGpsUartPtr->rec_buf),
                  index,
                  1,
                  &output);
	return output;
}

/** ****************************************************************************
 * @name peekGPSmsgHeader search and return GPS message header
 * @brief No error checking on the range of index or start of sentence word is
 *        performed.
 * @param [in] index represents the byte offset into the FIFO to look at.
 * @param [in] GPSData - data structure containng the message data
 * @retval the message header.
 ******************************************************************************/
unsigned long peekGPSmsgHeader(uint16_t      index,
                               GpsData_t     *GPSData)
{
    uint8_t       header[MAX_HEADER_LEN];
    int           i;
    int           j;
    int           headerLength;
	unsigned long GPSHeader;

    headerLength = GPSData->GPSMsgSignature.GPSheaderLength;
    COM_buf_copy (&(gGpsUartPtr->rec_buf),
                  index,
                  headerLength,
                  (uint8_t *)&header);
    GPSHeader = 0;
    for (i = 0, j = headerLength - 1; i < headerLength; i++, j--) {
        GPSHeader |= header[i] << (j * 8);
    }
	return GPSHeader;
}

/** ****************************************************************************
 * @name retrieveGpsMsg search for start of GPS message header
 * @brief COM_buf_get does a check for underflow the returns the message or
 *        nothing
 * @param [in] numBytes - to retrieve.
 * @param [in] GPSData - data structure containing the message information
 * @param [out] outBuffer - pointer to the output buffer
 * @retval The number of bytes left in the buffer. Or Zero - finished or
 *         buffer had exactly the number of bytes in the complete messsage
 ******************************************************************************/
int16_t retrieveGpsMsg(uint16_t      numBytes,
					   GpsData_t *GPSData,
					   uint8_t       *outBuffer)
{
	BOOL status = 0;
	status = COM_buf_get(&(gGpsUartPtr->rec_buf),
                         outBuffer,
                         numBytes);
	if (status == 0)
		return 0; // bad [0]
	else // return number of bytes left
		return gGpsUartPtr->rec_buf.bytes_in_buffer;
}

/** ****************************************************************************
 * @name findHeader search for start of GPS message header
 * @brief Does a simple check for the start byte then peeks at enough bytes
 *        to compare with signature header.
 * @param [in] numInBuff -  bytes available in input buffer.
 * @param [in] GPSData - data structure containing the message information
 * @retval The number of bytes left in the buffer including the header. Can
 *         be less than the complete messsage
 ******************************************************************************/
int16_t findHeader(uint16_t      numInBuff,
                   GpsData_t     *GPSData)
{
    uint8_t  buf[MAX_HEADER_LEN];
	uint8_t  byte;
	uint32_t header = 0;
	uint8_t  exit   = 0;

	do {
		COM_buf_copy_byte (&(gGpsUartPtr->rec_buf),
					       0, // index
					       &byte);
		if (byte == GPSData->GPSMsgSignature.startByte) {
			COM_buf_copy (&(gGpsUartPtr->rec_buf),
                  1, // index
                  GPSData->GPSMsgSignature.GPSheaderLength - 1,
                  (uint8_t *)&buf);
			header = byte <<  (GPSData->GPSMsgSignature.GPSheaderLength - 1) * 8;
			switch (GPSData->GPSMsgSignature.GPSheaderLength) {
				case 2: // SiRF 0xa0a2
					header |= (buf[0]);
					break;
				case 3: // NMEA "$GP", NovAtel binary 0xAA4412
					header |=  (buf[0] << 8) | (buf[1]);
					break;
			}
            if ( header == GPSData->GPSMsgSignature.GPSheader ) {
				exit = 1;
			} else {
				COM_buf_delete_byte(&(gGpsUartPtr->rec_buf));
				numInBuff--;
			}
		} else {
			COM_buf_delete_byte(&(gGpsUartPtr->rec_buf));
			numInBuff--;
		}
    } while ( (exit == 0) && (numInBuff > 0) );
	return numInBuff;
}

/** ****************************************************************************
 * @name writeGps send data to the GPS
 * @param [in] data - to go out
 * @param [in] len - length in bytes
 * @retval N/A
 * @details gConfiguration.userBehavior.bit.useGPS (use USART) determines
 *          which pipeline to send command through
 ******************************************************************************/
int writeGps(char     *data,
             uint16_t len)
{
    uint16_t i;
    if (UseExtUSARTGps() == true ) { // using external USART
        do {
            DebugSerialPutChar(data[i]);
            i++;
        } while (i < len);

    } else { // internal Gps UART
        uart_write(gUartChannel,
                   gGpsUartPtr);
        COM_buf_add(&(gGpsUartPtr->xmit_buf),
                   (unsigned char*) data,
                   len);
    }
    return 0;
}

