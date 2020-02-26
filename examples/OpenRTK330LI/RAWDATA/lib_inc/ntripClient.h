#ifndef _NTRIPCLIENT_H_
#define _NTRIPCLIENT_H_

#include <stdio.h>
#include "lwip/err.h"
#include "lwip/api.h"
#include "lwip/opt.h"
#include "LwipComm.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "RingBuffer.h"

// ntrip buffer size
#define NTRIP_TX_BUFSIZE 2000
#define NTRIP_RX_BUFSIZE 2000

// ntrip client start
#define NTRIP_START_OFF 0
#define NTRIP_START_ON  1

// ntrip base stream
#define BSAE_OFF 0
#define BSAE_ON  1

// ntrip client state
typedef enum
{
    NTRIP_STATE_OFF                 = 0,
    NTRIP_STATE_CONNECT             = 1,
    NTRIP_STATE_REQUEST             = 2,
    NTRIP_STATE_INTERACTIVE         = 3,
    NTRIP_STATE_TIMEOUT             = 4,
    NTRIP_STATE_LINK_DOWN           = 5
} ntrip_client_state_enum_t;


extern struct netconn *Ntrip_client;
extern uint8_t NTRIP_client_start;
extern uint8_t NTRIP_client_state;
extern uint8_t NTRIP_base_stream;

extern FIFO_Type ntrip_tx_fifo;
extern FIFO_Type ntrip_rx_fifo;
extern uint8_t ntripTxBuf[NTRIP_TX_BUFSIZE];
extern uint8_t ntripRxBuf[NTRIP_RX_BUFSIZE];
extern uint16_t ntripTxLen;
extern uint16_t ntripRxLen;
extern uint32_t ntripStreamCount;

#define NTRIP_STREAM_CONNECTED_MAX_COUNT 20

void Fill_EthLocalRTKPacketPayload(uint8_t* payload, uint16_t *payloadLen);
void Fill_EthCloudRTKPacketPayload(uint8_t* payload, uint16_t *payloadLen);

err_t NTRIP_GET_Data(uint8_t *rxBuf, uint16_t *rxLen);
err_t NTRIP_Recieve_Data(FIFO_Type* fifo);
err_t NTRIP_Write_Data(uint8_t *txBuf, uint16_t txLen, uint8_t apiflags);
void ntrip_link_down(void);
void NTRIP_interface(void);

#endif