/*******************************************************************************
* File Name          : dma_memcpy.h
* Author             : Daich
* Revision           : 1.0
* Date               : 30/09/2019
* Description        : dma_memcpy.h
*
* HISTORY***********************************************************************
* 30/09/2019  |                                             | Daich
* Description: create

* 15/10/2019  |                                             | Daich
* Description: modify dma_memcpy_test
*******************************************************************************/
#ifndef _DMA_MEMCPY_H_
#define _DMA_MEMCPY_H_
//#pragma once
#include <stdint.h>

typedef enum
{
  DMA_MEM_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled */
  DMA_MEM_STATE_READY             = 0x01U,  /*!< DMA initialized and ready for use   */
  DMA_MEM_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing              */
  DMA_MEM_STATE_TIMEOUT           = 0x03U,  /*!< DMA timeout state                   */
  DMA_MEM_STATE_ERROR             = 0x04U,  /*!< DMA error state                     */
  DMA_MEM_STATE_ABORT             = 0x05U,  /*!< DMA Abort state                     */
}DMA_MEM_STATE_E;

DMA_MEM_STATE_E get_dma_mem_state();
void dma_memcpy(uint32_t dst_address, uint32_t src_address, uint32_t data_len);
void dma_memcpy_init(void);
int dma_memcpy_test(void);
int buffer_cmp(const uint32_t* pBuffer, uint32_t* pBuffer1, uint16_t BufferLength);
#endif
