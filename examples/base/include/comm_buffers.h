/** ***************************************************************************
 * @file comm_buffers.h for the uart driver(extern_port), for use by the system
 * @author lfera
 * @date   2010-08-18 10:05:16 -0700 (Wed, 18 Aug 2010)
 * @rev 16069
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef COMM_BUFFERS_H
#define COMM_BUFFERS_H
#include "dmu.h"
// cir_buf defined in port_def.h
extern unsigned int COM_buf_bytes_available (cir_buf *buf_struc);
extern unsigned int COM_buf_headroom (cir_buf *buf_struc);
extern unsigned int COM_buf_delete (cir_buf *buf_struc, unsigned int pop_cnt);
extern unsigned int COM_buf_delete_byte (cir_buf *circBuf );
extern BOOL COM_buf_copy (cir_buf *buf_struc,unsigned int buf_index,unsigned int cnt,unsigned char *buf_out);
extern BOOL COM_buf_copy_byte (cir_buf *circBuf, unsigned int  bufIndex, unsigned char *bufOut);
extern BOOL COM_buf_add(cir_buf *buf_struc,unsigned char *buf,unsigned int cnt);
extern BOOL COM_buf_get(cir_buf *buf_struc,unsigned char *buf,unsigned int cnt);
extern void COM_buf_init(port_struct *port,unsigned char *tx_buf,unsigned char *rx_buf,unsigned int rx_size,unsigned int tx_size);

#endif
