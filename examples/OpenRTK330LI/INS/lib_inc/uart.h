/*******************************************************************************
* File Name          : uart.h
* Author             : Daich
* Revision           : 1.0
* Date               : 10/10/2019
* Description        : uart drive head file
*
* HISTORY***********************************************************************
* 10/10/2019  |                                             | Daich
* Description: create
* 16/10/2019  |                                             | Daich
* Description: //#define UART_BLOCK to save memory
*******************************************************************************/
#ifndef _UART_H_
#define _UART_H_
//#pragma once
#include "driver.h"
#include "RingBuffer.h"
#include <stdint.h>
#ifndef BAREMETAL_OS
	#include "osapi.h"
	#include "osresources.h"
#else
	#include "bare_osapi.h"
#endif
#include "stm32f4xx_hal.h"
//#define OFFLINE_DEBUG

#if 0
#define UART_USER_BASE                USART1
#define UART_BT_BASE                  USART2
#define UART_GPS_BASE                 USART3
#define UART_DEBUG_BASE               UART5
#endif
#define UART_USER_BASE                UART5         //TODO:
#define UART_BT_BASE                  USART2
#define UART_GPS_BASE                 USART3
#define UART_DEBUG_BASE               USART1

extern UART_HandleTypeDef huart_debug;
extern UART_HandleTypeDef huart_user;
extern UART_HandleTypeDef huart_bt;
extern UART_HandleTypeDef huart_gps;
extern DMA_HandleTypeDef hdma_usart_user_rx;
extern DMA_HandleTypeDef hdma_usart_user_tx;
extern DMA_HandleTypeDef hdma_uart_debug_rx;
extern DMA_HandleTypeDef hdma_uart_debug_tx;
extern DMA_HandleTypeDef hdma_usart_gps_rx;
extern DMA_HandleTypeDef hdma_usart_gps_tx;
extern DMA_HandleTypeDef hdma_usart_bt_rx;
extern DMA_HandleTypeDef hdma_usart_bt_tx;

extern FIFO_Type uart_gps_rx_fifo;
extern FIFO_Type uart_debug_rx_fifo;
extern FIFO_Type uart_bt_rx_fifo;
extern FIFO_Type uart_user_rx_fifo;

#define USER_UART_DMA_FIFO
#ifdef USER_UART_DMA_FIFO
#define UART_TX_FIFO_MANAGE_NUM  1

#define DMA_TX_FIFO_BUF_SIZE     2048

typedef struct _uart_tx_fifo
{
    FIFO_Type uart_tx_fifo;
    uint32_t frame_num;
    uint32_t data_total_num;
    uint32_t is_data_available;
    uint8_t is_dma_busy;    //TODO:
}uart_tx_dma_fifo_s;
#endif



//#define UART_BLOCK 
typedef enum {
    UART_USER = 0x00,  
    UART_BT = 0x01,  
    UART_GPS = 0x02,  
	UART_DEBUG = 0x03,
    UART_MAX,
} uart_port_e;

struct uart_config_t {
    int uart_base_addr;
    uint32_t rec_buff_size;
    uint8_t* rec_buff;
    DMA_HandleTypeDef* hdma_usart_rx;
    DMA_HandleTypeDef* hdma_usart_tx;
};

typedef struct {
    uart_port_e uart_num; 
    int baudrate;
#ifdef UART_BLOCK
    osSemaphoreId rx_sem;
    osSemaphoreId tx_sem;   
#endif 
#ifndef BAREMETAL_OS
    osSemaphoreId uart_idle_sem;
#endif
    bool init_flag;
	FIFO_Type* uart_rx_fifo;
	UART_HandleTypeDef* huart;
    DMA_HandleTypeDef* hdma_usart_rx;
    DMA_HandleTypeDef* hdma_usart_tx;
} uart_obj_t;

int uart_read_bytes(uart_port_e uart_num, uint8_t* buf, uint32_t len, TickType_t ticks_to_wait);
int uart_driver_install(uart_port_e uart_num, FIFO_Type* uart_rx_fifo,UART_HandleTypeDef* huart,int baudrate);
rtk_ret_e uart_driver_delete(uart_port_e uart_num);
int uart_write_bytes(uart_port_e uart_num, const char* src, size_t size, bool is_wait);
void update_fifo_in(uart_port_e uart_num);
rtk_ret_e uart_sem_wait(uart_port_e uart_num,uint32_t millisec);
#endif