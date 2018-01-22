/** ***************************************************************************
 * @file can.c the definitions of basic functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifdef MEMSIC_CAN
#include <stdio.h>
#include <stdlib.h>

#include "salvodefs.h"
#include "stm32f2xx.h"
#include "timer.h"
#include "dmu.h"
#include "xbowsp_algorithm.h"
#include "xbowsp_init.h"
#include "can.h"
#include "boardDefinition.h"
#include "debug.h" 
#include "sae_j1939.h"

void (*gCANTxCompleteCallback)(void) = NULL;
void (*gCANRxCompleteCallback)(void) = NULL;

int     gCANSleep = FALSE;
uint32_t canRxIntCounter = 0;
uint32_t canStartDetectRxIntCounter = 0;


uint8_t get_can_sleep(CAN_TypeDef* CANx) 
{ 
      return gCANSleep; 
}

uint8_t sleep_can(CAN_TypeDef* CANx) 
{
  return CAN_Sleep(CANx);
}

uint8_t wake_up_can(CAN_TypeDef* CANx) 
{
  return CAN_WakeUp(CANx);
}

void CAN_Set_BTR(_ECU_BAUD_RATE br, CAN_InitTypeDef* CAN_InitStructure)
{
  switch (br) {
      case _ECU_500K:
        CAN_InitStructure->CAN_Prescaler = 10;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_4tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_1tq;
        break;
      case _ECU_250K:
        CAN_InitStructure->CAN_Prescaler = 24;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_3tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_1tq;
        break;
      case _ECU_125K:
        CAN_InitStructure->CAN_Prescaler = 24;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_7tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_2tq;
        break;
      case _ECU_1000K:
        CAN_InitStructure->CAN_Prescaler = 2;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_7tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_2tq;
        break;
      default:
        break;
  }
  
  return;
}


uint8_t _CAN_Init(CAN_TypeDef* CANx, uint8_t mode)
{
    CAN_InitTypeDef  CAN_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
        
    CAN_DeInit(CANx);
    
    CAN_StructInit(&CAN_InitStructure);
        
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = mode;
   
    
    CAN_InitStructure.CAN_SJW = 0;
    if ( gConfiguration.ecuBaudRate > 3)
      CAN_Set_BTR(_ECU_500K, &CAN_InitStructure);
    else
      CAN_Set_BTR((_ECU_BAUD_RATE)gConfiguration.ecuBaudRate, &CAN_InitStructure);
//    if (gEcuConfig.baudRate == _ECU_250K) {
//        CAN_InitStructure.CAN_SJW = 0;
//        CAN_InitStructure.CAN_Prescaler = 24;
//        CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
//        CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
//    } else if (gEcuConfig.baudRate == _ECU_500K) {
//      //10, 6, 1, 400
//      //14, 4, 1, 350
//        CAN_InitStructure.CAN_Prescaler = 10;
//        CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
//        CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
////    } else if (gEcuConfig.baudRate == _ECU_1000K) {
////        CAN_InitStructure.CAN_Prescaler = 2;
////        CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq;
////        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
//    } else { // 125K
//        CAN_InitStructure.CAN_Prescaler = 24;
//        CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq;
//        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
//    }
  
      
    if (CANx == CAN1) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); 
    
        RCC_AHB1PeriphClockCmd(CAN1_RX_GPIO_CLK, ENABLE);
        RCC_AHB1PeriphClockCmd(CAN1_TX_GPIO_CLK, ENABLE);
    
        GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_RX_SOURCE, GPIO_AF_CAN1);
        GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_TX_SOURCE, GPIO_AF_CAN1);
    
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
        GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
        GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);
                   
        CAN_Init(CAN1, &CAN_InitStructure);
#ifndef FL        
         /// Enable the CAN1 global interrupt, CAN1_TX_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_TX_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x9;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
         
         /// Enable the CAN1 global interrupt, CAN1_RX0_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX0_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xa;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
         
         // Enable the CAN2 global interrupt, CAN2_RX1_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX1_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xa;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x2;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
#endif        
        
    } else {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); 
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); 
    
        RCC_AHB1PeriphClockCmd(CAN2_RX_GPIO_CLK, ENABLE);
        RCC_AHB1PeriphClockCmd(CAN2_TX_GPIO_CLK, ENABLE);
    
        GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_RX_SOURCE, GPIO_AF_CAN2);
        GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_TX_SOURCE, GPIO_AF_CAN2);
    
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN;
        GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
        GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);
        
        CAN_Init(CAN2, &CAN_InitStructure);
#ifdef FL        
         /// Enable the CAN2 global interrupt, CAN2_TX_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_TX_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x9;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
         
         /// Enable the CAN2 global interrupt, CAN2_RX0_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xa;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
         
         /// Enable the CAN2 global interrupt, CAN2_RX1_IRQn
//         NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX1_IRQn;
//         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xa;
//         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x2;
//         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
//         NVIC_Init( &NVIC_InitStructure );
#endif
    }
    
    CAN_DBGFreeze(CANx, DISABLE);
    
    return CAN_NO_ERROR;
}

void _CAN_Init_Filter(void)
{
  CAN_FilterInitTypeDef FILTER_InitStructure;
  uint8_t data[2];
  uint32_t mask = 0x1fffff00;
  
  // version filter
  data[0] = (uint8_t)(SAE_J1939_CONTROL_PRIORITY << 5) | (SAE_J1939_PDU_FORMAT_SOFTWARE_VERSION >> 5);
  data[1] = (uint8_t)(SAE_J1939_PDU_FORMAT_SOFTWARE_VERSION << 3) | (SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION >> 5);
  FILTER_InitStructure.CAN_FilterIdHigh = (data[0] << 8) | data[1];
  
  data[0] = (uint8_t)(SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION << 3);
  data[1] = (uint8_t)(MEMSIC_CAN_IDE << 2) | (MEMSIC_CAN_RTR << 1);
  FILTER_InitStructure.CAN_FilterIdLow = (data[0] << 8) | data[1];
  
  FILTER_InitStructure.CAN_FilterMaskIdHigh = mask >> 16;
  FILTER_InitStructure.CAN_FilterMaskIdLow = mask & 0xffff;
  FILTER_InitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;
  FILTER_InitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  FILTER_InitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  FILTER_InitStructure.CAN_FilterActivation = ENABLE;
  FILTER_InitStructure.CAN_FilterNumber = 5;
  
  CAN_FilterInit(&FILTER_InitStructure);
  
  // ecu id filter
  data[0] = (uint8_t)(SAE_J1939_CONTROL_PRIORITY << 5) | (SAE_J1939_PDU_FORMAT_ECU >> 5);
  data[1] = (uint8_t)(SAE_J1939_PDU_FORMAT_ECU << 3) | (SAE_J1939_GROUP_EXTENSION_ECU >> 5);
  FILTER_InitStructure.CAN_FilterIdHigh = (data[0] << 8) | data[1];
  
  data[0] = (uint8_t)(SAE_J1939_GROUP_EXTENSION_ECU << 3);
  data[1] = (uint8_t)(MEMSIC_CAN_IDE << 2) | (MEMSIC_CAN_RTR << 1);
  FILTER_InitStructure.CAN_FilterIdLow = (data[0] << 8) | data[1];
  FILTER_InitStructure.CAN_FilterNumber = 4;
  CAN_FilterInit(&FILTER_InitStructure);
  
  // Configuration
  data[0] = (uint8_t)(SAE_J1939_CONTROL_PRIORITY << 5) | (SAE_J1939_PDU_FORMAT_GLOBAL >> 5);
  data[1] = (uint8_t)(SAE_J1939_PDU_FORMAT_GLOBAL << 3) | (0x5f >> 5);
  FILTER_InitStructure.CAN_FilterIdHigh = (data[0] << 8) | data[1];
  
  data[0] = (uint8_t)(0x5f << 3);
  data[1] = (uint8_t)(MEMSIC_CAN_IDE << 2) | (MEMSIC_CAN_RTR << 1);
  FILTER_InitStructure.CAN_FilterIdLow = (data[0] << 8) | data[1];
  FILTER_InitStructure.CAN_FilterNumber = 3;
  FILTER_InitStructure.CAN_FilterMaskIdHigh = mask >> 16;
  FILTER_InitStructure.CAN_FilterMaskIdLow = 0;
  FILTER_InitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  CAN_FilterInit(&FILTER_InitStructure);
  
  // request 
  data[0] = (uint8_t)(SAE_J1939_REQUEST_PRIORITY << 5) | (SAE_J1939_PDU_FORMAT_REQUEST >> 5);
  data[1] = (uint8_t)(SAE_J1939_PDU_FORMAT_REQUEST << 3) | (SAE_J1939_PDU_FORMAT_GLOBAL >> 5);
  FILTER_InitStructure.CAN_FilterIdHigh = (data[0] << 8) | data[1];
  
  data[0] = (uint8_t)(SAE_J1939_PDU_FORMAT_GLOBAL << 3);
  data[1] = (uint8_t)(MEMSIC_CAN_IDE << 2) | (MEMSIC_CAN_RTR << 1);
  FILTER_InitStructure.CAN_FilterIdLow = (data[0] << 8) | data[1];
  FILTER_InitStructure.CAN_FilterNumber = 2;
  CAN_FilterInit(&FILTER_InitStructure);
  
  // address claim
  data[0] = (uint8_t)(SAE_J1939_REQUEST_PRIORITY << 5) | (SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM >> 5);
  data[1] = (uint8_t)(SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM << 3) | (SAE_J1939_GROUP_EXTENSION_ACK >> 5);
  FILTER_InitStructure.CAN_FilterIdHigh = (data[0] << 8) | data[1];
  
  data[0] = (uint8_t)(SAE_J1939_GROUP_EXTENSION_ACK << 3);
  data[1] = (uint8_t)(MEMSIC_CAN_IDE << 2) | (MEMSIC_CAN_RTR << 1);
  FILTER_InitStructure.CAN_FilterIdLow = (data[0] << 8) | data[1];
  FILTER_InitStructure.CAN_FilterNumber = 1;
  CAN_FilterInit(&FILTER_InitStructure);
  
  // BANK0 of PS
  data[0] = (uint8_t)(SAE_J1939_CONTROL_PRIORITY << 5) | (SAE_J1939_PDU_FORMAT_GLOBAL >> 5);
  data[1] = (uint8_t)(SAE_J1939_PDU_FORMAT_GLOBAL << 3) | (SAE_J1939_GROUP_EXTENSION_BANK0 >> 5);
  FILTER_InitStructure.CAN_FilterIdHigh = (data[0] << 8) | data[1];
  
  data[0] = (uint8_t)(SAE_J1939_GROUP_EXTENSION_BANK0 << 3);
  data[1] = (uint8_t)(MEMSIC_CAN_IDE << 2) | (MEMSIC_CAN_RTR << 1);
  FILTER_InitStructure.CAN_FilterIdLow = (data[0] << 8) | data[1];
  FILTER_InitStructure.CAN_FilterNumber = 6;
  CAN_FilterInit(&FILTER_InitStructure);
  
  // BANK1 of PS
  data[0] = (uint8_t)(SAE_J1939_CONTROL_PRIORITY << 5) | (SAE_J1939_PDU_FORMAT_GLOBAL >> 5);
  data[1] = (uint8_t)(SAE_J1939_PDU_FORMAT_GLOBAL << 3) | (SAE_J1939_GROUP_EXTENSION_BANK1 >> 5);
  FILTER_InitStructure.CAN_FilterIdHigh = (data[0] << 8) | data[1];
  
  data[0] = (uint8_t)(SAE_J1939_GROUP_EXTENSION_BANK1 << 3);
  data[1] = (uint8_t)(MEMSIC_CAN_IDE << 2) | (MEMSIC_CAN_RTR << 1);
  FILTER_InitStructure.CAN_FilterIdLow = (data[0] << 8) | data[1];
   FILTER_InitStructure.CAN_FilterNumber = 7;
  CAN_FilterInit(&FILTER_InitStructure);
  
  CAN_SlaveStartBank(28);
  
  return;
 
}

void _CAN_Configure(void (*callback1)(void), void(*callback2)(void))
{
   gCANTxCompleteCallback = callback1;
   gCANRxCompleteCallback = callback2;
   
   return;
}

void _CAN_Init_IT(CAN_TypeDef* CANx)
{
  uint32_t int_bits = 0;
  
  int_bits = CAN_IT_TME | CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FMP1 | CAN_IT_FF1 | CAN_IT_FOV1;
  
  CAN_ITConfig(CANx, int_bits, ENABLE);
  
  return;
}

void _CAN_IT_Enable(CAN_TypeDef* CANx, uint32_t int_bits)
{
   CAN_ITConfig(CANx, int_bits, ENABLE);
  
  return;
}

void _CAN_IT_Disable(CAN_TypeDef* CANx, uint32_t int_bits)
{
  CAN_ITConfig(CANx, int_bits, DISABLE);
  
  return;
}

void CAN1_TX_IRQHandler(void)
{
  ITStatus ItRslt;
  
  OSDisableHook();
  
  ItRslt = CAN_GetITStatus(CAN1, CAN_IT_TME);
  
  if(ItRslt == SET) {
    gCANTxCompleteCallback();
    CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
  }
  
  OSEnableHook(); 
  
  return;
}

void CAN1_RX0_IRQHandler(void)
{
#ifdef SAEJ1939
  ITStatus ItRslt;
  uint8_t fifoPending;
    
  OSDisableHook();
  
  canRxIntCounter++;
  ItRslt =  CAN_GetITStatus(CAN1, CAN_IT_FMP0);
  if (ItRslt == SET) {
    fifoPending = CAN_MessagePending(CAN1, 0);
    do {
        if (gEcuInst.curr_rx_desc->rx_pkt_ready == DESC_OCCUPIED)
             break;
        CAN_Receive(CAN1, 0, &(gEcuInst.curr_rx_desc->rx_buffer));
        CAN_FIFORelease(CAN1, 0);
        gCANRxCompleteCallback();
        fifoPending--;
    } while (fifoPending > 0);
    
    CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
  }
  
  OSEnableHook(); 
#endif
  return;
}
  
void CAN1_RX1_IRQHandler(void)
{
#ifdef SAEJ1939
  ITStatus ItRslt;
  uint8_t fifoPending;
  
  OSDisableHook();
  
  ItRslt =  CAN_GetITStatus(CAN1, CAN_IT_FMP1);
  if (ItRslt == SET) {
    fifoPending = CAN_MessagePending(CAN1, 1);
    do {
        if (gEcuInst.curr_rx_desc->rx_pkt_ready == DESC_OCCUPIED)
             break;
        CAN_Receive(CAN1, 1, &(gEcuInst.curr_rx_desc->rx_buffer));
        CAN_FIFORelease(CAN1, 1);
        gCANRxCompleteCallback();
        fifoPending--;
    } while (fifoPending > 0);
    
    CAN_ClearITPendingBit(CAN1, CAN_IT_FF1);
  }
  
  OSEnableHook();
#endif
  
  return;
}

//FL first version to CAN2
void CAN2_TX_IRQHandler(void)
{
#ifdef FL
  ITStatus ItRslt;
  
  OSDisableHook();
  
  ItRslt = CAN_GetITStatus(CAN2, CAN_IT_TME);
  
  if(ItRslt == SET) {
    gCANTxCompleteCallback();
    CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
  }
  
  OSEnableHook(); 
#endif
  return;
}

void CAN2_RX0_IRQHandler(void)
{
#ifdef FL
  ITStatus ItRslt;
  uint8_t fifoPending;
    
  OSDisableHook();
  
  ItRslt =  CAN_GetITStatus(CAN2, CAN_IT_FMP0);
  if (ItRslt == SET) {
    fifoPending = CAN_MessagePending(CAN2, 0);
    do {
        if (gEcuInst.curr_rx_desc->rx_pkt_ready == DESC_OCCUPIED)
             break;
        CAN_Receive(CAN2, 0, &(gEcuInst.curr_rx_desc->rx_buffer));
        CAN_FIFORelease(CAN2, 0);
        gCANRxCompleteCallback();
        fifoPending--;
    } while (fifoPending > 0);
    
    CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);
  }
  
  OSEnableHook(); 
#endif
  
  return;
}
  
void CAN2_RX1_IRQHandler(void)
{
#ifdef FL
  ITStatus ItRslt;
  uint8_t fifoPending;
  
  OSDisableHook();
  
  ItRslt =  CAN_GetITStatus(CAN2, CAN_IT_FMP1);
  if (ItRslt == SET) {
    fifoPending = CAN_MessagePending(CAN2, 1);
    do {
        if (gEcuInst.curr_rx_desc->rx_pkt_ready == DESC_OCCUPIED)
             break;
        CAN_Receive(CAN2, 1, &(gEcuInst.curr_rx_desc->rx_buffer));
        CAN_FIFORelease(CAN2, 1);
        gCANRxCompleteCallback();
        fifoPending--;
    } while (fifoPending > 0);
    
    CAN_ClearITPendingBit(CAN2, CAN_IT_FF1);
  }
  
  OSEnableHook(); 
#endif  
  return;
}

void InitCommunication_UserCAN()
{
  
  _CAN_Init(CAN1, CAN_Mode_Normal); 
  
  _CAN_Init_Filter();
  
  _CAN_Init_IT(CAN1);
 
  
  return;
}

_ECU_BAUD_RATE CAN_Detect_Baudrate(CAN_TypeDef* CANx, _ECU_BAUD_RATE rate)
{
   CAN_InitTypeDef  CAN_InitStructure;
   _ECU_BAUD_RATE found_rate;
     
  if (CANx != CAN1) 
    return _ECU_1000K;
  
  
  if (rate == _ECU_1000K)
      return rate;
  
  CAN_StructInit(&CAN_InitStructure);
        
  CAN_InitStructure.CAN_ABOM = ENABLE;
  CAN_InitStructure.CAN_AWUM = ENABLE;
  CAN_InitStructure.CAN_TXFP = ENABLE;
  CAN_InitStructure.CAN_SJW = 0;
  CAN_Set_BTR(rate, &CAN_InitStructure);
   
  CAN_Init(CAN1, &CAN_InitStructure);
    
  DelayMs(MEMSIC_CAN_DETECT_TIME);
    
  if (canRxIntCounter > canStartDetectRxIntCounter + 2) {
      found_rate = rate;
      return found_rate;
  }
   
  return _ECU_1000K;
}

#endif
