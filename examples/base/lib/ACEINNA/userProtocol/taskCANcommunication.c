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

#include "configureGPIO.h"
#include "bsp.h" // for definition of LED4

extern uint32_t can_bus_heart_beat;
extern uint32_t tim5_heart_beat;

extern uint32_t userCommunicationType;
extern void setUserCommunicationType(uint32_t);
uint32_t CanCounter = 0;

extern EcuConfigurationStruct gEcuConfig;

void TaskCANCommunication(void)
{
    setUserCommunicationType(0);
    ExternPortInit();
   
    InitCommunication_UserCAN();
    if (!(gEcuConfigPtr->ecu_name.words)) {
      gEcuConfigPtr->ecu_name.bits.function = MEMSIC_SAE_J1939_FUNCTION;
      gEcuConfigPtr->ecu_name.bits.manufacture_code = MEMSIC_SAE_J1939_MANUFACTURE_CODE;
      gEcuConfigPtr->ecu_name.bits.identity_number = gCalibration.serialNumber & 0x1fffff;
    }
    sae_j1939_initialize();
    
    // Main loop for the task
    while( 1 )
    {   
       OS_WaitBinSem(BINSEM_CAN_DATA, OS_TICKS_PER_SECOND);
#ifdef MEMSIC_CAN_HOST
          ecu_host_test(19);
#else
          // ecu process
          ecu_process();
          CanCounter++;
          
          switch (gEcuConfig.config_changed & _ECU_CONFIG_MASK) {
              case _ECU_CONFIG_PACKET_TYPE:
                gEcuConfig.config_changed &= ~_ECU_CONFIG_PACKET_TYPE;
                break;
              case _ECU_CONFIG_DIGITAL_FILTER:
//                if (gEcuConfig.rate_cut_off == 5)
//                  gConfiguration.analogFilterClocks[2] = 
//                gConfiguration.analogFilterClocks[1]
//                gConfiguration.analogFilterClocks[1]
                gEcuConfig.config_changed &= ~_ECU_CONFIG_DIGITAL_FILTER;
                break;
              case _ECU_CONFIG_ORIENTATION:
                gEcuConfig.config_changed &= ~_ECU_CONFIG_ORIENTATION;
                gConfiguration.orientation.all = *(uint16_t *)&gEcuConfig.rate_orien;
                break;
              case _ECU_CONFIG_USER_BEHAVIOR:
                gEcuConfig.config_changed &= ~_ECU_CONFIG_USER_BEHAVIOR;
                break;
              case _ECU_CONFIG_ANGLE_ALARM:
                gEcuConfig.config_changed &= ~_ECU_CONFIG_ANGLE_ALARM;
                break;
              case _ECU_CONFIG_CONE_ALARM:
                gEcuConfig.config_changed &= ~_ECU_CONFIG_CONE_ALARM;
                break;
              case _ECU_CONFIG_ACCELERATION_PARAM:
                gEcuConfig.config_changed &= ~_ECU_CONFIG_ACCELERATION_PARAM;
                break;
              case _ECU_CONFIG_GROUP_EXTENSION_BANK:
                gEcuConfig.config_changed &= ~_ECU_CONFIG_GROUP_EXTENSION_BANK;
                break;
              default:
                break;
          }
          
          
        
#endif // _CAN_HOST

    }
}
#endif
