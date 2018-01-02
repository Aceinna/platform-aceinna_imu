/** ***************************************************************************
 * @file extern_port.c functions for general external port interface
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief Memsic ucb (Unified Code Base) external serial interface
 *****************************************************************************/
#include "stm32f2xx.h"
#include "dmu.h"
#include "ucb_packet.h"
#include "extern_port.h"
#include "extern_port_config.h"
#include "comm_buffers.h"
#include "xbowsp_generaldrivers.h"
#include "uart.h"
#include "driverGPS.h"

/// size of port-associated software circular buffers
#define PORT_0_RX_BUF_SIZE	 COM_BUF_SIZE // 512
#define PORT_0_TX_BUF_SIZE   COM_BUF_SIZE
#define PORT_1_RX_BUF_SIZE	 COM_BUF_SIZE
#define PORT_1_TX_BUF_SIZE   COM_BUF_SIZE

/// packet-type receive working buffer sizes
#define UCB_RX_WORKING_BUFFER_SIZE	128
#define CRM_RX_WORKING_BUFFER_SIZE	128

/// main loop period
extern const uint16_t MAIN_LOOP_PERIOD;		///< 10 milliseconds (approximately)

/// port behavior parameters
#define RX_POLL_PERIOD	MAIN_LOOP_PERIOD	///< milliseconds (approximately)
#define UCB_RX_TIMEOUT 	1000				///< milliseconds (approximately)
#define CRM_RX_TIMEOUT	1000				///< milliseconds (approximately)

static port_struct *portMap;

/// initial CRC seed to use for UCB packet CRC
static const UcbPacketCrcType UCB_PACKET_CRC_INITIAL_SEED = CRC_CCITT_INITIAL_SEED;

/// for forcing state machine reset whenever initialization has been performed
static BOOL stateReset = FALSE;
port_struct gPort0; // User com
port_struct gPort1; // GPS

/** ****************************************************************************
 * @name:	GetGpsExternPortAndChannel
 *
 * @param [out] uartChannel - channel number
 * @param [out] gpsUart - portnumber
 * @retval: N/A
 ******************************************************************************/
void GetGpsExternPortAndChannel(unsigned int* uartChannel,
                                port_struct** gpsUart)
{
    *uartChannel = 1;
    *gpsUart     = &gPort1;
}

/** ****************************************************************************
 * @name ExternPortInit
 * @brief Initializes the serial port and maps logical ports.
 * Trace:
 * [SDD_EXT_SERIAL_INIT_USAGE <-- SRC_EXT_PORT_INIT]
 * [SDD_EXT_SERIAL_INIT_MAP <-- SRC_EXT_PORT_INIT]
 * [SDD_EXT_SERIAL_INIT_OFF <-- SRC_EXT_PORT_INIT]
 *
 * @param N/A
 * @brief
 * Static variables affected:
 *     BAUD_TABLE[]
 *     physicalPort
 *     portMap
 *     portNumMap
 * @retval N/A
 ******************************************************************************/
void ExternPortInit (void)
{
    /// port-associated software rx and tx circular buffers
	static uint8_t port0_rx [PORT_0_RX_BUF_SIZE], port0_tx [PORT_0_TX_BUF_SIZE];
	static uint8_t port1_rx [PORT_1_RX_BUF_SIZE], port1_tx [PORT_1_TX_BUF_SIZE];

    /// set baud rates from configuration
    gPort0.hw.baud = baudEnumToBaudRate(gConfiguration.baudRateUser);

    /// initialize software circular buffers comm_buffers.c User com
    COM_buf_init(&gPort0,
                 port0_tx,
                 port0_rx,
                 PORT_0_RX_BUF_SIZE,
                 PORT_0_TX_BUF_SIZE);
    // GPS PORT
    COM_buf_init(&gPort1,
                 port1_tx,
                 port1_rx,
                 PORT_1_RX_BUF_SIZE,
                 PORT_1_TX_BUF_SIZE);

	/// initialize the physical UART ports
	uart_init(0, &(gPort0.hw));
	uart_init(1, &(gPort1.hw));

    portMap    =  &gPort0;

	/// reset all state machines (used for re-initialization)
	stateReset = TRUE;
} /* end ExternPortInit */


typedef enum {
    RESET_STATE,
    SYNC_STATE,
    PACKET_TYPE_STATE,
    PAYLOAD_LENGTH_STATE,
    DATA_STATE,
    CRC_STATE,
    CHECK_CRC_STATE,
    PACKET_DONE_STATE
} StateType;

/** ****************************************************************************
 * @name HandleUcbRx
 * @brief handles received ucb packets
 * Trace:
 *	[SDD_UCB_TIMEOUT_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_PACKET_CRC <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_CONVERT_DATA <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_STORE_DATA <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_UNKNOWN_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_CRC_FAIL_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_VALID_PACKET <-- SRC_HANDLE_UCB_RX]
 *
 * @param [in] port - logical port type
 * @param [out] packetPtr - UCB packet to read the packet into
 * @retval TRUE if a full packet has been seen (can fail CRC)
 *         FALSE if needing more to fill in a packet
 ******************************************************************************/
BOOL HandleUcbRx (ExternPortTypeEnum port,
                  UcbPacketStruct    *ptrUcbPacket)

{
	static StateType state          = RESET_STATE ;
    static BOOL		 timeoutRunning =  FALSE ; ///< receive timeout timer
    static uint16_t	 timeoutCount   =  0 ;
    /// for assembling packets
    static uint8_t 	 workingBuffer [UCB_RX_WORKING_BUFFER_SIZE];
    	   uint16_t	 bufferIndex;
    static uint16_t  bytesToRead    = 0; ///< from each port buffer
    static uint16_t  syncBytesFound = 0; ///< current working buffer
    uint8_t          byteCount;          ///< general use byte counter
    static uint8_t   dataCount;          /// words to read from port buffer
    static UcbPacketCrcType packetCrc;   ///< received packet
    BOOL                    packetDone = FALSE; ///< complete packet received?

    /// increment timeout timer if neccessary, check if timeout limit was reached
    if (timeoutRunning == TRUE) {
        if (++timeoutCount >= (UCB_RX_TIMEOUT / RX_POLL_PERIOD))	{
            timeoutCount = 0;
            ptrUcbPacket->packetType = UCB_ERROR_TIMEOUT;
            state = PACKET_DONE_STATE;
	 	}
	 }

    /// handle current state
    if ((state == RESET_STATE) ||
    	(stateReset == TRUE)) {
    	timeoutCount   = 0;
    	bytesToRead    = 0; ///< bytes to read counts
    	syncBytesFound = 0; ///< number of synchronization bytes found
    	state          = SYNC_STATE;
    	stateReset     = FALSE;
    } /* end if */

// FIXME - these should be in a callback table with the common code segments
// seperted into functions to make maintainence easier. - only have to change
// code in one place
    if (state == SYNC_STATE) {
    	/// Rx bytes to read?
        bytesToRead = COM_buf_bytes_available(&(portMap->rec_buf));

        if (bytesToRead > 0) {
            /// bytes to read doesn't exceed allocated buffer size?
            if (bytesToRead < UCB_RX_WORKING_BUFFER_SIZE) {
                /// copy all available bytes from Rx buffer
                COM_buf_copy(&(portMap->rec_buf),
                               0,
                               bytesToRead,
                             &(workingBuffer[0]));
                } else {
                /// copy from the Rx buffer maximum allocated to buffer
                COM_buf_copy(&(portMap->rec_buf),
                             0,
                             UCB_RX_WORKING_BUFFER_SIZE,
                             &(workingBuffer[0]));
            }

            /// loop through all bytes until preamble is found, stop looping if found
            for (bufferIndex = 0;
                 (bufferIndex < bytesToRead) &&
                 (syncBytesFound < UCB_SYNC_LENGTH);
                 ++bufferIndex) {

                if (syncBytesFound < UCB_SYNC_LENGTH)	{ ///< not all preamble sync bytes have been found
                    if (workingBuffer[bufferIndex] == UCB_SYNC[syncBytesFound]) {
                        timeoutRunning = TRUE;
                        ++syncBytesFound;
                    } else { /** not all preamble bytes appeared in a contiguous
                                     sequence, not a preamble sync sequence */
                    timeoutRunning = FALSE;
                    timeoutCount   = 0;
                    syncBytesFound = 0;
                    }
                }
            }
            /// all preamble sync bytes have been found
            if (syncBytesFound == UCB_SYNC_LENGTH) {
                syncBytesFound = 0;
                state          = PACKET_TYPE_STATE;
            }
            /// remove all bytes from Rx buffer up through current byte
            COM_buf_delete(&(portMap->rec_buf),
                           bufferIndex);
        }
	}

    if (state == PACKET_TYPE_STATE) {
    	/// available bytes to read?
        bytesToRead = COM_buf_bytes_available(&(portMap->rec_buf));

    	/// greater than size of packet type field?
    	if (bytesToRead >= UCB_PACKET_TYPE_LENGTH) {
            /// read available bytes
		    COM_buf_get(&(portMap->rec_buf),
                        &(workingBuffer[0]),
                        UCB_PACKET_TYPE_LENGTH);

            /// place packet type in packet structure
            ptrUcbPacket->packetType = UcbPacketBytesToPacketType(&(workingBuffer[0]));

            /// check for invalid or non-input packet type
            if ((ptrUcbPacket->packetType == UCB_ERROR_INVALID_TYPE) ||
                (UcbPacketIsAnInputPacket(ptrUcbPacket->packetType) == FALSE)) {

                /// place invalid packet type at beginning of data field
                for (byteCount = 0; byteCount < UCB_PACKET_TYPE_LENGTH; ++byteCount) {
                    ptrUcbPacket->payload[byteCount] = workingBuffer[byteCount];
                }
                state = PACKET_DONE_STATE;
            } else {
                /// compute running CRC
                packetCrc = UcbPacketCalculateCrc(&(workingBuffer[0]),
                                                   UCB_PACKET_TYPE_LENGTH,
                                                   UCB_PACKET_CRC_INITIAL_SEED);
                state = PAYLOAD_LENGTH_STATE;
            }
        }
    }

    if (state == PAYLOAD_LENGTH_STATE) {
    	/// available bytes to read?
        bytesToRead = COM_buf_bytes_available(&(portMap->rec_buf));

        /// greater than size of packet type field?
        if (bytesToRead >= UCB_PAYLOAD_LENGTH_LENGTH) {
            /// read in available bytes
            COM_buf_get(&(portMap->rec_buf),
                        &(workingBuffer[0]),
                        UCB_PAYLOAD_LENGTH_LENGTH);

            /// place packet length in packet structure
            ptrUcbPacket->payloadLength = UcbPacketBytesToPayloadLength(&(workingBuffer[0]));

            /// place payload length in data byte counter
            dataCount = UcbPacketBytesToPayloadLength(&(workingBuffer[0]));
            /// compute running CRC
            packetCrc = UcbPacketCalculateCrc(&(workingBuffer[0]),
                                                UCB_PAYLOAD_LENGTH_LENGTH,
                                                packetCrc);
            state = DATA_STATE;
        }
    }

    if (state == DATA_STATE) {
    	if (dataCount > 0) {	/// some packets have no data fields, skip this state if so
    		/// available bytes to read?
            bytesToRead = COM_buf_bytes_available(&(portMap->rec_buf));

    		/// greater than number of remaining payload field bytes?
    		if (bytesToRead >= dataCount) {
                /// place data bytes in packet structure
                COM_buf_get(&(portMap->rec_buf),
                            ptrUcbPacket->payload,
                            dataCount);
                /// compute running CRC
                packetCrc = UcbPacketCalculateCrc(ptrUcbPacket->payload,
                                                  dataCount,
                                                  packetCrc);
    			state = CRC_STATE;
    		}
    	} else {
    		state = CRC_STATE;
    	}
    }

    if (state == CRC_STATE) {
    	/// available bytes to read?
        bytesToRead = COM_buf_bytes_available(&(portMap->rec_buf));

    	/// greater than number of remaining payload field bytes?
    	if (bytesToRead >= UCB_CRC_LENGTH) {
            /// read in available bytes
            COM_buf_get(&(portMap->rec_buf),
                        &(workingBuffer[0]),
                        UCB_CRC_LENGTH);
            /// compute running CRC
            packetCrc = UcbPacketCalculateCrc(&(workingBuffer[0]),
                                               UCB_CRC_LENGTH,
                                               packetCrc);
            state = CHECK_CRC_STATE;
        }
    }

    if (state == CHECK_CRC_STATE) {
        if (packetCrc == 0) {	/// CRC pass?
    		state = PACKET_DONE_STATE;
    	} else {	/// CRC fail
    		/// copy packet type to payload
    		UcbPacketPacketTypeToBytes(ptrUcbPacket->packetType,
                                       ptrUcbPacket->payload);

    		ptrUcbPacket->packetType = UCB_ERROR_CRC_FAIL;
    		state                    = PACKET_DONE_STATE;
    	}
    	/** prepare for next time ExternPortRx is called in case there are bytes
            in the buffer but none in hardware */
	    bytesToRead = COM_buf_bytes_available(&(portMap->rec_buf));
    }

    if (state == PACKET_DONE_STATE) {
    	timeoutRunning = FALSE;
    	packetDone     = TRUE;
    	state          = RESET_STATE;
    }

	return packetDone;
}
/* end HandleUcbRx */

/** ****************************************************************************
 * @name HandleUcbTx
 * @brief builds a UCB packet and then triggers transmission of it. Packet:
 *  Preamble = 0x5555
 *  Packet Type 0x####
 *  Length 0x##
 *  payload (uint8_t)data[Length]
 *  CRC 0x####
 * Trace: [SDD_UCB_PROCESS_OUT <-- SRC_UCB_OUT_PKT]
 * @param [in] port - port type UCB or CRM
 * @param [in] packetPtr -- buffer structure with payload, type and size
 * @retval valid packet in packetPtr TRUE
 ******************************************************************************/
void HandleUcbTx (ExternPortTypeEnum port,
                  UcbPacketStruct *ptrUcbPacket)

{
#if ((UCB_PACKET_TYPE_LENGTH + UCB_PAYLOAD_LENGTH_LENGTH) > UCB_CRC_LENGTH)
	uint8_t          data [UCB_PACKET_TYPE_LENGTH + UCB_PAYLOAD_LENGTH_LENGTH];
#else
    uint8_t          data [UCB_CRC_LENGTH];
#endif
	UcbPacketCrcType crc;

	/// get byte representation of packet type, index adjust required since sync
    /// isn't placed in data array
	UcbPacketPacketTypeToBytes(ptrUcbPacket->packetType,
                               &(data[UCB_PACKET_TYPE_INDEX - UCB_SYNC_LENGTH]));

	/// get byte representation of packet length, index adjust required since
    /// sync isn't placed in data array
	UcbPacketPayloadLengthToBytes(ptrUcbPacket->payloadLength,
                                  &(data[UCB_PAYLOAD_LENGTH_INDEX - UCB_SYNC_LENGTH]));

	/// place sync bytes in buffer 0x5555 length 2
	COM_buf_add(&(portMap->xmit_buf),
               (uint8_t *)UCB_SYNC,
               UCB_SYNC_LENGTH);

	/// place packet type (uint16_t) and payload length (uint8_t) bytes in buffer
    COM_buf_add(&(portMap->xmit_buf),
                data,
               (UCB_PACKET_TYPE_LENGTH + UCB_PAYLOAD_LENGTH_LENGTH));

	/// place payload bytes into buffer
	COM_buf_add(&(portMap->xmit_buf),
               (uint8_t *)ptrUcbPacket->payload,
               ptrUcbPacket->payloadLength);

	/// calculate running CRC on packet type and payload length bytes
	crc = UcbPacketCalculateCrc(data,
                                (UCB_PACKET_TYPE_LENGTH + UCB_PAYLOAD_LENGTH_LENGTH),
                                UCB_PACKET_CRC_INITIAL_SEED);

	/// calculate running CRC on payload bytes
	if (ptrUcbPacket->payloadLength > 0) {
		crc = UcbPacketCalculateCrc(ptrUcbPacket->payload,
                                    ptrUcbPacket->payloadLength,
                                    crc);
	}

	/// get byte representation of CRC
	UcbPacketCrcToBytes(crc, data);
	/// place CRC bytes into buffer
	COM_buf_add(&(portMap->xmit_buf),
                data,
                UCB_CRC_LENGTH);

    uart_write(0, portMap); // uart.c run transmit cycle on port
}
/* end HandleUcbTx */


/** ****************************************************************************
 * @name ExternPortWaitOnTxIdle
 * @brief Drain all pending output on all port types
 * Trace: [SDD_EXT_PORT_DRAIN <-- SRC_EXT_PORT_DRAIN]
 *        [SDD_WATCHDOG <-- SRC_EXT_PORT_DRAIN]
 * @param N/A
 * @retval N/A
 *******************************************************************************/
void ExternPortWaitOnTxIdle (void)
{
    static uint16_t channel = 0;
    kick_dog(); /// prevent watchdog timeout

	if (&gPort0 != 0) {
		while ( bytes_remaining(channel, &gPort0) > 0 ) {
            uart_write(0, portMap); // uart.c run transmit cycle on port
		}
	}
    kick_dog(); /// prevent watchdog timeout
}
