/** ***************************************************************************
 * @file dbc_file.h defines the format of DBC file
 * @brief Setting DBC file
 * @Author Feng
 * @date   Dec, 2017
 * @brief  Copyright (c) 2017, 2018 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef _DBC_FILE_H_
#define _DBC_FILE_H_
   
#define  DBC_RATE_FACTOR             0.007
#define  DBC_RATE_OFFSET             250.00
#define  DBC_ACCEL_FACTOR            0.001
#define  DBC_ACCEL_OFFSET            25.00

typedef union {
    struct {
      uint16_t   can_id    : 11;
      uint16_t   reserved  : 5;      
    } b;
    
    uint16_t r;
} DBC_IDENTIFIER_FIELD;

typedef union {
  struct {
    uint64_t  x_value     : 16;
    uint64_t  y_value     : 16;
    uint64_t  z_value     : 16;
    uint64_t  counter     : 4;
    uint64_t  reserved    : 4;
    uint64_t  csum        : 8;
  } b;
  
  uint64_t r;
} DBC_SG_PAYLOAD;


#define  ACEINNA_DBC_MESSAGE_STATUS_ID                 90
#define  ACEINNA_DBC_MESSAGE_RATE_ID                   91
#define  ACEINNA_DBC_MESSAGE_ACCEL_ID                  92

#define  DBC_MAX_TX_DESC                               32
#define  DBC_PAYLOAD_LEN                               8
    
struct dbc_tx_desc {
  uint8_t                     tx_payload_len;
  DESC_STATE                  tx_pkt_ready;
  DBC_IDENTIFIER_FIELD        tx_identifier;
  CanTxMsg                    tx_buffer;

  struct dbc_tx_desc          *next;  
};

extern void dbc_initialize();
extern void dbc_process();
extern void dbc_transmit_isr(void);

#endif