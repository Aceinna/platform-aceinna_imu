/** ***************************************************************************
 * @file   taskCANCommunication.c
 * @Author
 * @date   Aug, 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * End user communication with the DMU380:
 * - Nav-View (UART) maximum rate of 100 Hz (due to wait in the function below)
 *    handled by the Memsic ucb (Unified Code Base) - handle_packet, send_packet,
 *    extern_port.c, ucb_packet.c comm_buffers.c
 * - SPI communication is an external interrupt driven (asynchronous) bus
 ******************************************************************************/
#ifdef MEMSIC_CAN
#include "salvodefs.h"
#include "stm32f2xx.h"
#include "dmu.h"

#include "debug_usart.h"
#include "debug.h"

#include "extern_port.h"
#include "ucb_packet.h"
#include "uart.h"
#include "timer.h"
#include "xbowsp_algorithm.h"
#include "xbowsp_generaldrivers.h"
#include "xbowsp_init.h"
#include "boardDefinition.h"

   
// for can interface
#include "can.h"
#include "sae_j1939.h"
#include "dbc_file.h"

#include "configureGPIO.h"
#include "bsp.h" // for definition of LED4
   
#define ADDRESS_CLAIM_RETRY                 5

extern uint32_t can_bus_heart_beat;
extern uint32_t tim5_heart_beat;

extern uint32_t userCommunicationType;
extern void setUserCommunicationType(uint32_t);
uint32_t CanCounter = 0;


_ECU_BAUD_RATE baudRate; 

void TaskCANCommunication(void)
{
    CAN_InitTypeDef  CAN_InitStructure;
    static int8_t retry_time = MEMSIC_CAN_BAUD_RATE_RETRY; 
    _ECU_BAUD_RATE result;
     
    setUserCommunicationType(0);
    ExternPortInit();

    InitCommunication_UserCAN();
#ifdef SAEJ1939
       
    if (!(gEcuConfigPtr->ecu_name.words)) {
      gEcuConfigPtr->ecu_name.bits.function = MEMSIC_SAE_J1939_FUNCTION;
      gEcuConfigPtr->ecu_name.bits.manufacture_code = MEMSIC_SAE_J1939_MANUFACTURE_CODE;
      gEcuConfigPtr->ecu_name.bits.identity_number = gCalibration.serialNumber & 0x1fffff;
    }
    
    sae_j1939_initialize();
    
    baudRate = gEcuConfig.baudRate;
#else
    dbc_initialize();
#endif
        
    // Main loop for the task
    while( 1 )
    {   
       OS_WaitBinSem(BINSEM_CAN_DATA, OS_TICKS_PER_SECOND);
       
#ifdef SAEJ1939  
       if (gEcuInst.state == _ECU_BAUDRATE_DETECT) {
           canStartDetectRxIntCounter =  canRxIntCounter;
      
           result = CAN_Detect_Baudrate(CAN1, baudRate);
    
           if (result != _ECU_1000K) {
               gEcuConfig.baudRate = result;
               
                CAN_StructInit(&CAN_InitStructure);
        
               CAN_InitStructure.CAN_ABOM = ENABLE;
               CAN_InitStructure.CAN_AWUM = ENABLE;
               CAN_InitStructure.CAN_TXFP = ENABLE;
               CAN_InitStructure.CAN_SJW = 0;
               CAN_Set_BTR(gEcuConfig.baudRate, &CAN_InitStructure);
    
               CAN_Init(CAN1, &CAN_InitStructure);
               gEcuInst.state = _ECU_CHECK_ADDRESS;               
           }
           else {
             if (retry_time-- > 0) {
               baudRate++;
               continue;
             }
             
             CAN_DeInit(CAN1);
    
             CAN_StructInit(&CAN_InitStructure);
        
             CAN_InitStructure.CAN_ABOM = ENABLE;
             CAN_InitStructure.CAN_AWUM = ENABLE;
             CAN_InitStructure.CAN_TXFP = ENABLE;
             CAN_InitStructure.CAN_Mode = CAN_Mode_Silent;
    
             CAN_InitStructure.CAN_SJW = 0;
             CAN_Set_BTR(gEcuConfig.baudRate, &CAN_InitStructure);
    
             CAN_Init(CAN1, &CAN_InitStructure);
           }
    
      }
  

      if ((gEcuInst.state == _ECU_CHECK_ADDRESS) || (gEcuInst.state == _ECU_WAIT_ADDRESS)) {
           if (CanCounter++ > ADDRESS_CLAIM_RETRY)
               gEcuInst.state = _ECU_READY;
           send_address_claim(&gEcuInst);
      }
#ifdef FL // saej1939 host process
          ecu_host_test(19);
#endif
          // ecu process
          ecu_process();
          
          if (gEcuConfig.config_changed & _ECU_CONFIG_PACKET_TYPE) {
              gConfiguration.canPacketType = gEcuConfig.packet_type;
              gEcuConfig.config_changed &= ~_ECU_CONFIG_PACKET_TYPE;
          }
          
          if (gEcuConfig.config_changed & _ECU_CONFIG_PACKET_RATE) {
              gConfiguration.CanOdr = gEcuConfig.packet_rate;
              gEcuConfig.config_changed &= ~_ECU_CONFIG_PACKET_RATE;
          }
          
          if (gEcuConfig.config_changed & _ECU_CONFIG_DIGITAL_FILTER) {
                if (gEcuConfig.rate_cut_off == 2)
                  gConfiguration.analogFilterClocks[2] = 18750;
                else if (gEcuConfig.rate_cut_off == 5)
                  gConfiguration.analogFilterClocks[2] = 8035;
                else if (gEcuConfig.rate_cut_off == 10)
                  gConfiguration.analogFilterClocks[2] = 2411;
                else if (gEcuConfig.rate_cut_off == 20)
                  gConfiguration.analogFilterClocks[2] = 1741;
                else if (gEcuConfig.rate_cut_off == 25)
                  gConfiguration.analogFilterClocks[2] = 1205;
                else if (gEcuConfig.rate_cut_off == 40)
                  gConfiguration.analogFilterClocks[2] = 1;
                else
                  gConfiguration.analogFilterClocks[2] = 0;
                
                if (gEcuConfig.accel_cut_off == 2)
                  gConfiguration.analogFilterClocks[1] = 18750;
                else if (gEcuConfig.accel_cut_off == 5)
                  gConfiguration.analogFilterClocks[1] = 8035;
                else if (gEcuConfig.accel_cut_off == 10)
                  gConfiguration.analogFilterClocks[1] = 2411;
                else if (gEcuConfig.accel_cut_off == 20)
                  gConfiguration.analogFilterClocks[1] = 1741;
                else if (gEcuConfig.accel_cut_off == 25)
                  gConfiguration.analogFilterClocks[1] = 1205;
                else if (gEcuConfig.accel_cut_off == 40)
                  gConfiguration.analogFilterClocks[1] = 1;
                else
                  gConfiguration.analogFilterClocks[1] = 0;
                    
                gEcuConfig.config_changed &= ~_ECU_CONFIG_DIGITAL_FILTER;
          }
          
          if (gEcuConfig.config_changed & _ECU_CONFIG_ORIENTATION) {
                gEcuConfig.config_changed &= ~_ECU_CONFIG_ORIENTATION;
                gConfiguration.orientation.all = *(uint16_t *)&gEcuConfig.orien_bits;
          }
              
          if (gEcuConfig.config_changed & _ECU_CONFIG_GROUP_EXTENSION_BANK) {
                gConfiguration.algResetSaveCfgPs = (uint16_t)(gEcuConfig.alg_reset_ps << 8) | gEcuConfig.save_cfg_ps;
                gConfiguration.HardSoftBitPs = (uint16_t)(gEcuConfig.hardware_bit_ps << 8) | gEcuConfig.software_bit_ps;
                gConfiguration.statusPrPs = (uint16_t)(gEcuConfig.status_ps << 8) | gEcuConfig.packet_rate_ps;
                gConfiguration.PtDfPs = (uint16_t)(gEcuConfig.packet_type_ps << 8) | gEcuConfig.digital_filter_ps;
                gConfiguration.OrienUserBehvPs = (uint16_t)(gEcuConfig.orientation_ps << 8) | gEcuConfig.user_behavior_ps;
                gConfiguration.AngConeAlarmPs = (uint16_t)(gEcuConfig.angle_alarm_ps << 8) | gEcuConfig.cone_alarm_ps;
                gEcuConfig.config_changed &= ~_ECU_CONFIG_GROUP_EXTENSION_BANK;             
          }
#endif // SAEJ1939
          
#ifdef DBC_FILE
          dbc_process();
#endif

    }
}
#endif // MEMSIC_CAN
