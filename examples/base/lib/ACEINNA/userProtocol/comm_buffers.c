/** ***************************************************************************
 * @file comm_buffers.c
 * @author tdelong
 * @date   2011-02-08 22:36:46 -0800 (Tue, 08 Feb 2011)
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @version 17420
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * This is a set of routines to move data in and out of the communication
 * circular buffers. They will also report back the byte used and bytes available
 * in the buffer passed. These are common routines used on both bootloader and DUP
 *****************************************************************************/
#include <stdint.h>

#include "dmu.h"
#include "extern_port_config.h"
#include "port_def.h"
#include "comm_buffers.h"
#include "debug.h"

#define GOOD TRUE
#define BAD  FALSE

/** ****************************************************************************
 * @name COM_buf_bytes_available
 * @brief returns the number of bytes IN the circular buffer the buf_struc
 *        points to.
 * Trace:
 * [SDD_COM_BUFFER_BYTES_02 <-- SRC_COM_BUFFER_BYTES]
 * [SDD_COM_BUFFER_BYTES_03 <-- SRC_COM_BUFFER_BYTES]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @retval number of bytes in the buffer
 ******************************************************************************/
unsigned int COM_buf_bytes_available (cir_buf *circBuf)
{
    return circBuf->bytes_in_buffer;
}   /* end of COM_buf_bytes_available */

/** ****************************************************************************
 * @name COM_buf_headroom
 * @brief returns the number of bytes AVAILABLE in the circular
 * buffer the buf_struc points
 * Trace:
 * [SDD_COM_BUF_SPACE_02 <-- SRC_COM_BUF_SPACE]
 * [SDD_COM_BUF_SPACE_03 <-- SRC_COM_BUF_SPACE]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @retval number of bytes available in the buffer.
 ******************************************************************************/
unsigned int COM_buf_headroom (cir_buf *circBuf)
{
	return (circBuf->buf_size - circBuf->bytes_in_buffer);
}   /* end of COM_buf_headroom */

/** ****************************************************************************
 * @name COM_buf_delete
 * @brief This routine moves the tail pointer by - adding popCnt to the output
 *        pointer of the circular buffer. If popCnt exceeds the number of bytes
 *        in the buffer nothing will be removed.
 * Trace:
 * [SDD_COM_BUF_POP_01 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_02 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_03 <-- SRC_COM_BUF_POP]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @param [in] delCnt - number of bytes to remove from buffer
 * @retval number of bytes popped.
@details Note: This does NOT decrement the bytes available
 ******************************************************************************/
unsigned int COM_buf_delete (cir_buf      *circBuf,
                             unsigned int delCnt)
{
	unsigned int newOut;

	if ( circBuf->bytes_in_buffer >= delCnt )  {
		circBuf->bytes_in_buffer -= delCnt;
		newOut      = circBuf->buf_outptr + delCnt;
		if(newOut >= circBuf->buf_size) {
			newOut -= circBuf->buf_size;
		}
		circBuf->buf_outptr = newOut;
		return delCnt;
    } // else
    return 0; // UNDERFLOW - do nothing
}  /* end of COM_buf_delete */

/** ****************************************************************************
 * @name COM_buf_delete_byte
 * @brief This routine moves the tail pointer by - adding popCnt to the output
 *        pointer of the circular buffer. If popCnt exceeds the number of bytes
 *        in the buffer nothing will be removed.
 *
 * Trace:
 * [SDD_COM_BUF_POP_01 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_02 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_03 <-- SRC_COM_BUF_POP]
 *
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @param [in] delCnt - number of bytes to remove from buffer
 * @retval number of bytes popped 1 or 0.
 ******************************************************************************/
unsigned int COM_buf_delete_byte (cir_buf *circBuf )
{
	unsigned int newOut;

	if ( circBuf->bytes_in_buffer >= 1 )  {
		circBuf->bytes_in_buffer -= 1;
		newOut      = circBuf->buf_outptr + 1;
		if(newOut >= circBuf->buf_size) {
			newOut -= circBuf->buf_size;
		}
		circBuf->buf_outptr = newOut;
		return 1;
    } // else
    return 0; // UNDERFLOW - do nothing
}  /* end of COM_buf_delete_byte */

/** ****************************************************************************
 * @name COM_buf_copy copy out the number of bytes requested without removing
 *       them from the buffer.
 * @brief This routine uses the outptr and the bufIndex to point to the start
 *        of a string of bytes. If asked for more bytes than are in the buffer
 *        an error will be generated. The bytes read use the bufOut pointer for
 *        the starting add.
 * Trace:
 * [SDD_COM_BUF_READ_01 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_02 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_03 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_04 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_05 <-- SRC_COM_BUF_READ]
 * @param [in] circBuf - pointer to the circular buffer structure
 * @param [in] bufIndex - buffer index
 * @param [in] cnt  - number of bytes output to bufOut pointer
 * @param [in] bufOut - points to the output string of chars
 * @retval status returns a BOOL either GOOD or BAD
 ******************************************************************************/
BOOL COM_buf_copy (cir_buf       *circBuf,
                   unsigned int  bufIndex,
                   unsigned int  cnt,
                   unsigned char *bufOut)
{
	unsigned int strPtr;
	unsigned int i;

	strPtr = circBuf->buf_outptr + bufIndex;
    // if there is more bytes available than requested
	if(circBuf->bytes_in_buffer >= cnt + bufIndex) {
		for (i = 0; i < cnt; i++) {
			if (strPtr >= circBuf->buf_size) {
				strPtr -= circBuf->buf_size;
			}
			*bufOut = *(circBuf->buf_add + strPtr++);
		 	bufOut++;
	    }
		return GOOD;
	}
    return BAD; // UNDEREFLOW
}   /* end of COM_buf_copy */

/** ****************************************************************************
 * @name COM_buf_copy_byte copy out a byte without removing it from the buffer.
 * @brief This routine will use the outptr and the bufIndex to point to the start
 * of a string of charaters.  If asked for more characters than are in the
 * buffer an error will be generated. The bytes read use the bufOut pointer for
 * the starting add.
 * Trace:
 * [SDD_COM_BUF_READ_01 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_02 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_03 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_04 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_05 <-- SRC_COM_BUF_READ]
 * @param [in] circBuf - pointer to the circular buffer structure
 * @param [in] bufIndex - buffer index
 * @param [in] bufOut - points to the output string of chars
 * @retval status returns a BOOL either GOOD or BAD
 ******************************************************************************/
BOOL COM_buf_copy_byte (cir_buf       *circBuf,
						unsigned int  bufIndex,
						unsigned char *bufOut)
{
	unsigned int strPtr;

	strPtr = circBuf->buf_outptr + bufIndex;
    // if there are more bytes available than requested
	if(circBuf->bytes_in_buffer >= 1 + bufIndex) {
			if (strPtr >= circBuf->buf_size) {
				strPtr -= circBuf->buf_size;
			}
			*bufOut = *(circBuf->buf_add + strPtr++);
		 	bufOut++;
		return GOOD;
	}
    return BAD; // UNDEREFLOW
}   /* end of COM_buf_copy_byte */

/** ****************************************************************************
 * @name COM_buf_add - add the number of bytes requested to the circ buffer
 * @brief uses cnt to check the available space in the circular
 *        buffer. If there is enough room, write it into the buffer using the
 *        pointer passed to it. If there is not enough room in the buffer no
 *        data will be sent and an error will be returned
 * Trace:
 * [SDD_COM_BUF_IN_01 <-- SRC_COM_BUF_IN]
 * [SDD_COM_BUF_IN_02 <-- SRC_COM_BUF_IN]
 * [SDD_COM_BUF_IN_03 <-- SRC_COM_BUF_IN]
 * [SDD_COM_BUF_IN_04 <-- SRC_COM_BUF_IN]
 * @param [in] circBuf - pointer to the circular buffer structure
 * @param [in] *buf - pointer to input data
 * @param [in] cnt - number of bytes to put into the circular buffer
 * @param [out] circBuf.inptr in the buffer struct will be incremeted
 * @retval status returns a BOOL either GOOD or BAD
 ******************************************************************************/
BOOL COM_buf_add(cir_buf      *circBuf,
                unsigned char *buf,
                unsigned int  cnt)
{
	unsigned int i;

	if(COM_buf_headroom(circBuf) >= cnt) {
		circBuf->bytes_in_buffer += cnt;
		for (i = 0; i < cnt; i++) {
			if(circBuf->buf_inptr >= circBuf->buf_size) {
				circBuf->buf_inptr -= circBuf->buf_size;
			}
			*(circBuf->buf_add + (circBuf->buf_inptr++)) = *buf;
			buf++;
		}
        // move the input pointer
		if(circBuf->buf_inptr >= circBuf->buf_size) {
			circBuf->buf_inptr -= circBuf->buf_size;
		}
		return GOOD;
	} // else
      return BAD; // BAD = overflow
}   /* end of COM_buf_add */

/** ****************************************************************************
 * @name COM_buf_get copy and remove "Pop" the number of bytes requested
 * @brief Compares the number of bytes in cnt to the number of
 *        bytes in the buffer.  If the cnt is greater than the number of bytes
 *        in the buffer then no data will be moved and an error returned. If
 *        there is enough data in the circular buffer then cnt number of bytes
 *        will be moved to the location pointed to by the buf pointer.
 * Trace:
 * [SDD_COM_BUF_OUT_01 <-- SRC_COM_BUF_OUT]
 * [SDD_COM_BUF_OUT_02 <-- SRC_COM_BUF_OUT]
 * [SDD_COM_BUF_OUT_03 <-- SRC_COM_BUF_OUT]
 * [SDD_COM_BUF_OUT_04 <-- SRC_COM_BUF_OUT]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @param [in] *buf - pointer to the output location
 * @param [in] cnt - number of byte to pull from cir buffer.
 * @param [out] circBuf.outptr in the buffer is incremented
 * @retval status returns either GOOD [1] or BAD [0] set if not enough bytes in
 *         circular buffer.
 ******************************************************************************/
BOOL COM_buf_get(cir_buf       *circBuf,
                 unsigned char *buf,
                 unsigned int  cnt)
{
	unsigned int i;

	if(circBuf->bytes_in_buffer >= cnt) {
		circBuf->bytes_in_buffer -= cnt;
	    for (i = 0; i < cnt; i++) {
            if ( circBuf->buf_outptr >= circBuf->buf_size) {
                circBuf->buf_outptr -= circBuf->buf_size;
            } // copy the requested byte to the output buffer
              // and move the tail (output) pointer
			*buf = *(circBuf->buf_add + ( circBuf->buf_outptr++ ));
		 	buf++;
	    }  // handle roll over
	    if(circBuf->buf_outptr >= circBuf->buf_size) {
            circBuf->buf_outptr -= circBuf->buf_size;
        }
		return GOOD;
	} // else
//    printf("COMM Circular Buffer UNDERFLOW\r\n");
    return BAD;
}       /*end of COM_buf_get*/

/** ****************************************************************************
 * @name COM_buf_init
 * @brief initalizes the buffer related pointers, indexs buffer size. it also
 *        sets datatype and tx_int_flg to zero. These items are
 *        the only ones that are appropriate for this routine to initialize all
 *        others should be done by the subroutines which are related to them.
 *        The data type and hw_int_flg are set here to inhibit functions from
 *        running if not initialized.
 * Trace:
 * [SDD_COM_BUF_INIT_01 <-- SRC_COM_BUF_INIT]
 * [SDD_COM_BUF_INIT_02 <-- SRC_COM_BUF_INIT]
 * [SDD_COM_BUF_INIT_03 <-- SRC_COM_BUF_INIT]
 *
 * @param [in] port - pointer to the circular buffer structure
 * @param [in] *tx_buf - pointer to the transmit buffer.
 * @param [in] *rx_buf - pointer to the receive buffer.
 * @param [in] rx_size - buffer size
 * @param [in] tx_size - buffer size
 * Return value: N/A
 ******************************************************************************/
void COM_buf_init(port_struct   *port,
                  unsigned char *tx_buf,
                  unsigned char *rx_buf,
                  unsigned int  rx_size,
                  unsigned int  tx_size)
{
	port->rec_buf.buf_add    = rx_buf;
	port->rec_buf.buf_size   = rx_size;
	port->rec_buf.buf_inptr  = 0;
	port->rec_buf.buf_outptr = 0;

	port->xmit_buf.buf_add    = tx_buf;
	port->xmit_buf.buf_size   = tx_size;
	port->xmit_buf.buf_inptr  = 0;
	port->xmit_buf.buf_outptr = 0;

}  /*end of COM_buf_init*/
