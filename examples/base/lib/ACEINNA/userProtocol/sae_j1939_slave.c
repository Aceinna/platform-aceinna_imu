/** ***************************************************************************
 * @file sae_j1939_slave.c the definitions of basic functions
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
#ifdef MEMSIC_CAN_380
#include <stdio.h>
#include <stdlib.h>

#include "stm32f2xx.h"
#include "DLib_Product_string.h"
#include "dmu.h"
#include "timer.h"
#include "debug.h"
#include "xbowsp_init.h"
#include "xbowsp_fields.h"
#include "scaling.h"
#include "EKF_Algorithm.h"
#include "..\math\qmath.h"

#include "sae_j1939.h"

extern uint32_t can_bus_heart_beat;

ECU_INSTANCE * gEcu = &gEcuInst;

uint8_t memsic_j1939_send_software_version(void)
{
  struct sae_j1939_tx_desc *ver_desc;
  
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&ver_desc) == 0)
    return 0;
  
 
  ver_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  ver_desc->tx_payload_len = MEMSIC_SAE_J1939_VERSION_PACKET_LEN;
  ver_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  ver_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_SOFTWARE_VERSION;
  ver_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION;
  ver_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  ver_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  ver_desc->tx_buffer.ExtId = ((ver_desc->tx_identifier.r << 24) |
                               (ver_desc->tx_identifier.pdu_format << 16) |
                               (ver_desc->tx_identifier.pdu_specific << 8) |
                               (ver_desc->tx_identifier.source));
  ver_desc->tx_buffer.IDE = CAN_ID_EXT;
  ver_desc->tx_buffer.RTR = CAN_RTR_Data;
  ver_desc->tx_buffer.DLC = ver_desc->tx_payload_len;
  ver_desc->tx_buffer.Data[0] = (uint8_t)VERSION_MAJOR_NUM;
  ver_desc->tx_buffer.Data[1] = (uint8_t)VERSION_MINOR_NUM;
  ver_desc->tx_buffer.Data[2] = (uint8_t)VERSION_PATCH_NUM;
  ver_desc->tx_buffer.Data[3] = (uint8_t)VERSION_STAGE_NUM;
  ver_desc->tx_buffer.Data[4] = (uint8_t)VERSION_BUILD_NUM;
  
  return 1;
  
}

uint8_t memsic_j1939_send_ecu_id(void)
{
  struct sae_j1939_tx_desc *ecu_desc;
  
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&ecu_desc) == 0)
    return 0;
  
 
  ecu_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  ecu_desc->tx_payload_len = MEMSIC_SAE_J1939_ECU_PACKET_LEN;
  ecu_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  ecu_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_ECU;
  ecu_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_ECU;
  ecu_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  ecu_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  ecu_desc->tx_buffer.ExtId = ((ecu_desc->tx_identifier.r << 24) |
                               (ecu_desc->tx_identifier.pdu_format << 16) |
                               (ecu_desc->tx_identifier.pdu_specific << 8) |
                               (ecu_desc->tx_identifier.source));
  ecu_desc->tx_buffer.IDE = CAN_ID_EXT;
  ecu_desc->tx_buffer.RTR = CAN_RTR_Data;
  ecu_desc->tx_buffer.DLC = ecu_desc->tx_payload_len;
  memcpy((void *)ecu_desc->tx_buffer.Data, (void *)&(gEcuConfigPtr->ecu_name.words), MEMSIC_SAE_J1939_ECU_PACKET_LEN);
    
  return 1;
}


uint8_t memsic_j1939_send_algrst_cfgsave(ECU_ADDRESS_ENTRY *target, uint8_t alg_rst, uint8_t success)
{
  struct sae_j1939_tx_desc *algrst_desc;
  
  if (target->category != _ECU_MASTER)
      return 0;
      
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&algrst_desc) == 0)
    return 0;
  
  algrst_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  algrst_desc->tx_payload_len = MEMSIC_SAE_J1939_ALGO_RST_LEN;
  algrst_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  algrst_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  if (alg_rst)
      algrst_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET;
  else 
      algrst_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION;
  algrst_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  algrst_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  algrst_desc->tx_buffer.ExtId = ((algrst_desc->tx_identifier.r << 24) |
                                  (algrst_desc->tx_identifier.pdu_format << 16) |
                                  (algrst_desc->tx_identifier.pdu_specific << 8) |
                                  (algrst_desc->tx_identifier.source));
  algrst_desc->tx_buffer.IDE = CAN_ID_EXT;
  algrst_desc->tx_buffer.RTR = CAN_RTR_Data;
  algrst_desc->tx_buffer.DLC = algrst_desc->tx_payload_len;
  
  algrst_desc->tx_buffer.Data[0] = MEMSIC_SAE_J1939_RESPONSE;
  algrst_desc->tx_buffer.Data[1] = target->address;
  algrst_desc->tx_buffer.Data[2] = success;
  
  return 1;
}

uint8_t memsic_j1939_send_test_packet(uint8_t built_in_type, void * bit_fields)
{
  HARDWARE_TEST_PAYLOAD * hardware_test;
  SOFTWARE_TEST_PAYLOAD * software_test;
  STATUS_TEST_PAYLOAD   * status_test;
  struct sae_j1939_tx_desc * built_in_desc;
  
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&built_in_desc) == 0)
    return 0;
  
  built_in_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  built_in_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  built_in_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  if (built_in_type == MEMSIC_SAE_J1939_BUILTIN_HARDWARE) {
      built_in_desc->tx_payload_len = MEMSIC_SAE_J1939_HARDWARE_BITS_LEN;
      built_in_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_TEST_HARDWARE;
      hardware_test = (HARDWARE_TEST_PAYLOAD *)bit_fields;
      hardware_test->request = MEMSIC_SAE_J1939_RESPONSE;
            
      memcpy((void *)built_in_desc->tx_buffer.Data, (void *)hardware_test, MEMSIC_SAE_J1939_HARDWARE_BITS_LEN);      
  }
  else if (built_in_type == MEMSIC_SAE_J1939_BUILTIN_SOFTWARE) {
      built_in_desc->tx_payload_len = MEMSIC_SAE_J1939_SOFTWARE_BITS_LEN;
      built_in_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_TEST_SOFTWARE;
      software_test = (SOFTWARE_TEST_PAYLOAD *)bit_fields;
      software_test->request = MEMSIC_SAE_J1939_RESPONSE;
      
      memcpy((void *)built_in_desc->tx_buffer.Data, (void *)software_test, MEMSIC_SAE_J1939_SOFTWARE_BITS_LEN);       
  }
  else {
      built_in_desc->tx_payload_len = MEMSIC_SAE_J1939_STATUS_LEN;
      built_in_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_TEST_STATUS;
      status_test = (STATUS_TEST_PAYLOAD *)bit_fields;
      status_test->request = MEMSIC_SAE_J1939_RESPONSE;
     
      memcpy((void *)built_in_desc->tx_buffer.Data, (void *)status_test, MEMSIC_SAE_J1939_STATUS_LEN); 
  }
  
  built_in_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  built_in_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  built_in_desc->tx_buffer.ExtId = ((built_in_desc->tx_identifier.r << 24) |
                                    (built_in_desc->tx_identifier.pdu_format << 16) |
                                    (built_in_desc->tx_identifier.pdu_specific << 8) |
                                    (built_in_desc->tx_identifier.source));
  built_in_desc->tx_buffer.IDE = CAN_ID_EXT;
  built_in_desc->tx_buffer.RTR = CAN_RTR_Data;
  built_in_desc->tx_buffer.DLC = built_in_desc->tx_payload_len;
  
  return 1;
}

uint8_t memsic_j1939_send_packet_rate(ECU_ADDRESS_ENTRY *target, uint8_t odr)
{
  struct sae_j1939_tx_desc *pkt_rate_desc;
  
  if (target->category != _ECU_MASTER)
      return 0;
      
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&pkt_rate_desc) == 0)
    return 0;
  
  pkt_rate_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  pkt_rate_desc->tx_payload_len = MEMSIC_SAE_J1939_PACKET_RATE_LEN;
  pkt_rate_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  pkt_rate_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  pkt_rate_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_PACKET_RATE;
  pkt_rate_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  pkt_rate_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  pkt_rate_desc->tx_buffer.ExtId = ((pkt_rate_desc->tx_identifier.r << 24) |
                                    (pkt_rate_desc->tx_identifier.pdu_format << 16) |
                                    (pkt_rate_desc->tx_identifier.pdu_specific << 8) |
                                    (pkt_rate_desc->tx_identifier.source));
  pkt_rate_desc->tx_buffer.IDE = CAN_ID_EXT;
  pkt_rate_desc->tx_buffer.RTR = CAN_RTR_Data;
  pkt_rate_desc->tx_buffer.DLC = pkt_rate_desc->tx_payload_len;
  
  pkt_rate_desc->tx_buffer.Data[0] = target->address;
  pkt_rate_desc->tx_buffer.Data[1] = odr;
  
  return 1;
}

uint8_t memsic_j1939_send_packet_type(ECU_ADDRESS_ENTRY *target, uint8_t type)
{
  struct sae_j1939_tx_desc *pkt_type_desc;
  
  if (target->category != _ECU_MASTER)
      return 0;
      
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&pkt_type_desc) == 0)
    return 0;
  
  pkt_type_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  pkt_type_desc->tx_payload_len = MEMSIC_SAE_J1939_PACKET_TYPE_LEN;
  pkt_type_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  pkt_type_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  pkt_type_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_PACKET_TYPE;
  pkt_type_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  pkt_type_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  pkt_type_desc->tx_buffer.ExtId = ((pkt_type_desc->tx_identifier.r << 24) |
                                    (pkt_type_desc->tx_identifier.pdu_format << 16) |
                                    (pkt_type_desc->tx_identifier.pdu_specific << 8) |
                                    (pkt_type_desc->tx_identifier.source));
  pkt_type_desc->tx_buffer.IDE = CAN_ID_EXT;
  pkt_type_desc->tx_buffer.RTR = CAN_RTR_Data;
  pkt_type_desc->tx_buffer.DLC = pkt_type_desc->tx_payload_len;
  
  pkt_type_desc->tx_buffer.Data[0] = target->address;
  pkt_type_desc->tx_buffer.Data[1] = type;
  
  return 1;
}

uint8_t memsic_j1939_send_digital_filter(ECU_ADDRESS_ENTRY *target, uint8_t accel_cutoff, uint8_t rate_cutoff)
{
  struct sae_j1939_tx_desc *pkt_df_desc;
  
  if (target->category != _ECU_MASTER)
      return 0;
      
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&pkt_df_desc) == 0)
    return 0;
  
  pkt_df_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  pkt_df_desc->tx_payload_len = MEMSIC_SAE_J1939_DIGITAL_FILTER_LEN;
  pkt_df_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  pkt_df_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  pkt_df_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_DIGITAL_FILTER;
  pkt_df_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  pkt_df_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  pkt_df_desc->tx_buffer.ExtId = ((pkt_df_desc->tx_identifier.r << 24) |
                                   (pkt_df_desc->tx_identifier.pdu_format << 16) |
                                   (pkt_df_desc->tx_identifier.pdu_specific << 8) |
                                   (pkt_df_desc->tx_identifier.source));
  pkt_df_desc->tx_buffer.IDE = CAN_ID_EXT;
  pkt_df_desc->tx_buffer.RTR = CAN_RTR_Data;
  pkt_df_desc->tx_buffer.DLC = pkt_df_desc->tx_payload_len;
  
  pkt_df_desc->tx_buffer.Data[0] = target->address;
  pkt_df_desc->tx_buffer.Data[1] = accel_cutoff;
  pkt_df_desc->tx_buffer.Data[2] = rate_cutoff;
  
  return 1;
}

uint8_t memsic_j1939_send_orientation(ECU_ADDRESS_ENTRY *target, uint8_t *orien)
{
  struct sae_j1939_tx_desc *pkt_orien_desc;
  
  if (target->category != _ECU_MASTER)
      return 0;
      
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&pkt_orien_desc) == 0)
    return 0;
  
  pkt_orien_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  pkt_orien_desc->tx_payload_len = MEMSIC_SAE_J1939_ORIENTATION_LEN;
  pkt_orien_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  pkt_orien_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  pkt_orien_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_ORIENTATION;
  pkt_orien_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  pkt_orien_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  pkt_orien_desc->tx_buffer.ExtId = ((pkt_orien_desc->tx_identifier.r << 24) |
                                     (pkt_orien_desc->tx_identifier.pdu_format << 16) |
                                     (pkt_orien_desc->tx_identifier.pdu_specific << 8) |
                                     (pkt_orien_desc->tx_identifier.source));
  pkt_orien_desc->tx_buffer.IDE = CAN_ID_EXT;
  pkt_orien_desc->tx_buffer.RTR = CAN_RTR_Data;
  pkt_orien_desc->tx_buffer.DLC = pkt_orien_desc->tx_payload_len;
  
  pkt_orien_desc->tx_buffer.Data[0] = target->address;
  pkt_orien_desc->tx_buffer.Data[1] = orien[0];
  pkt_orien_desc->tx_buffer.Data[2] = orien[1];
  
  return 1;
}


uint8_t memsic_j1939_send_slope_sensor(SLOPE_SENSOR_2 * data)
{
  struct sae_j1939_tx_desc * slope_sensor_desc;
  
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&slope_sensor_desc) == 0)
    return 0;
  
  slope_sensor_desc->tx_pkt_type = SAE_J1939_DATA_PACKET;
  slope_sensor_desc->tx_identifier.control_bits.priority = SAE_J1939_SLOPE_PRIORITY;
  slope_sensor_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_DATA;
  slope_sensor_desc->tx_payload_len = MEMSIC_SAE_J1939_SLOPE_SENSOR2_LEN;
  slope_sensor_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR;
  slope_sensor_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  slope_sensor_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  slope_sensor_desc->tx_buffer.ExtId = ((slope_sensor_desc->tx_identifier.r << 24) |
                                        (slope_sensor_desc->tx_identifier.pdu_format << 16) |
                                        (slope_sensor_desc->tx_identifier.pdu_specific << 8) |
                                        (slope_sensor_desc->tx_identifier.source));
  slope_sensor_desc->tx_buffer.IDE = CAN_ID_EXT;
  slope_sensor_desc->tx_buffer.RTR = CAN_RTR_Data;
  slope_sensor_desc->tx_buffer.DLC = slope_sensor_desc->tx_payload_len;
  
  memcpy((void *)slope_sensor_desc->tx_buffer.Data, (void *)data, slope_sensor_desc->tx_payload_len);
  
  return 1;
}
    
uint8_t memsic_j1939_send_angular_rate(AUGULAR_RATE * data)
{
  struct sae_j1939_tx_desc * angular_rate_desc;
  
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&angular_rate_desc) == 0)
    return 0;
  
  angular_rate_desc->tx_pkt_type = SAE_J1939_DATA_PACKET;
  angular_rate_desc->tx_identifier.control_bits.priority = SAE_J1939_SLOPE_PRIORITY;
  angular_rate_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_DATA;
  angular_rate_desc->tx_payload_len = MEMSIC_SAE_J1939_ANGULAR_RATE_LEN;
  angular_rate_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE;
  angular_rate_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  angular_rate_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  angular_rate_desc->tx_buffer.ExtId = ((angular_rate_desc->tx_identifier.r << 24) |
                                        (angular_rate_desc->tx_identifier.pdu_format << 16) |
                                        (angular_rate_desc->tx_identifier.pdu_specific << 8) |
                                        (angular_rate_desc->tx_identifier.source));
  angular_rate_desc->tx_buffer.IDE = CAN_ID_EXT;
  angular_rate_desc->tx_buffer.RTR = CAN_RTR_Data;
  angular_rate_desc->tx_buffer.DLC = angular_rate_desc->tx_payload_len;
  
  memcpy((void *)angular_rate_desc->tx_buffer.Data, (void *)data, angular_rate_desc->tx_payload_len);
  
  return 1;
}
     
uint8_t memsic_j1939_send_acceleration(ACCELERATION_SENSOR * data)
{
  struct sae_j1939_tx_desc * acceleration_desc;
  
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&acceleration_desc) == 0)
    return 0;
  
  acceleration_desc->tx_pkt_type = SAE_J1939_DATA_PACKET;
  acceleration_desc->tx_identifier.control_bits.priority = SAE_J1939_ACCELERATION_PRIORITY;
  acceleration_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_DATA;
  acceleration_desc->tx_payload_len = MEMSIC_SAE_J1939_ACCELERATION_LEN;
  acceleration_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_ACCELERATION;
  acceleration_desc->tx_identifier.source = *(uint8_t *)gEcu->addr;
  acceleration_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  acceleration_desc->tx_buffer.ExtId = ((acceleration_desc->tx_identifier.r << 24) |
                                        (acceleration_desc->tx_identifier.pdu_format << 16) |
                                        (acceleration_desc->tx_identifier.pdu_specific << 8) |
                                        (acceleration_desc->tx_identifier.source));
  acceleration_desc->tx_buffer.IDE = CAN_ID_EXT;
  acceleration_desc->tx_buffer.RTR = CAN_RTR_Data;
  acceleration_desc->tx_buffer.DLC = acceleration_desc->tx_payload_len;
  
  memcpy((void *)acceleration_desc->tx_buffer.Data, (void *)data, acceleration_desc->tx_payload_len);
  
  return 1;
}

MEMSIC_J1939_PACKET_TYPE is_valid_j1939_rcv(struct sae_j1939_rx_desc *rx_desc)
{
  SAE_J1939_IDENTIFIER_FIELD *ident;
  MEMSIC_J1939_PACKET_TYPE result;
 
  if (rx_desc == NULL)
       return MEMSIC_J1939_INVALID_IDENTIFIER;

  ident = &(rx_desc->rx_identifier);
  
  if (!is_valid_sae_j1939_identifier(ident))
    return MEMSIC_J1939_INVALID_IDENTIFIER;
  
  result = is_data_packet(ident);
  if (result == MEMSIC_J1939_DATA)
     return MEMSIC_J1939_IGNORE;
  
  result = is_valid_address_claim(ident);
  if (result == MEMSIC_J1939_ADDRESS_CLAIM)
     goto end_j1939_rcv;
  
  result = is_valid_config_command(ident);
  if (result == MEMSIC_J1939_CONFIG)
    goto end_j1939_rcv;
  
  result = MEMSIC_J1939_REQUEST_PACKET;
  
end_j1939_rcv:
  
  return result;        
}

static void ecu_alg_reset(void)
{
  gAlgorithm.state = STABILIZE_SYSTEM;
  InitializeAlgorithmStruct(&gAlgorithm);
  
  return;
}

static uint8_t ecu_config_save(void)
{
    gConfiguration.ecuAddress = (uint16_t)gEcuConfigPtr->address;
    gConfiguration.ecuBaudRate = (uint16_t)gEcuConfigPtr->baudRate;
  
    gConfiguration.algResetSaveCfgPs = ((uint16_t)gEcuConfigPtr->alg_reset_ps << 8) | gEcuConfigPtr->save_cfg_ps;
    gConfiguration.HardSoftBitPs = ((uint16_t)gEcuConfigPtr->hardware_bit_ps << 8) | gEcuConfigPtr->software_bit_ps;
    gConfiguration.statusPrPs = ((uint16_t)gEcuConfigPtr->status_ps << 8) | gEcuConfigPtr->packet_rate_ps;
    gConfiguration.PtDfPs = ((uint16_t)gEcuConfigPtr->packet_type_ps << 8) | gEcuConfigPtr->digital_filter_ps;
    gConfiguration.OrienUserBehvPs = ((uint16_t)gEcuConfigPtr->orientation_ps << 8) | gEcuConfigPtr->user_behavior_ps;
    gConfiguration.AngConeAlarmPs = ((uint16_t)gEcuConfigPtr->angle_alarm_ps << 8) | gEcuConfigPtr->cone_alarm_ps;
  
    return WriteFieldData();
}

void ecu_command_set(void * command, uint8_t ps)
{ 
  if (command == NULL)
    return;
  
  switch (ps) {
  case SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET:
      if ((((COMMAND_SET_PAYLOAD *)command)->request == MEMSIC_SAE_J1939_REQUEST) &&
          (((COMMAND_SET_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr)) {
          ecu_alg_reset();
      }
      break;
  case SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION:
      if ((((COMMAND_SET_PAYLOAD *)command)->request == MEMSIC_SAE_J1939_REQUEST) &&
          (((COMMAND_SET_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr)) {
          ecu_config_save();
      }
      break;
  case SAE_J1939_GROUP_EXTENSION_PACKET_RATE:
    if (((RATE_CONFIG_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->packet_rate = ((RATE_CONFIG_PAYLOAD *)command)->odr;
      gEcuConfigPtr->config_changed = _ECU_CONFIG_PACKET_RATE;
      can_bus_heart_beat = TIM5_OUTPUT_DATA_RATE / gEcuConfig.packet_rate;
    }
    break;
  case SAE_J1939_GROUP_EXTENSION_PACKET_TYPE:
    if (((PACKET_TYPE_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->packet_type = ((PACKET_TYPE_PAYLOAD *)command)->type_bits.r;
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_PACKET_TYPE;
    }
    break;
  case SAE_J1939_GROUP_EXTENSION_DIGITAL_FILTER:
    if (((DIGITAL_FILTER_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->rate_cut_off = ((DIGITAL_FILTER_PAYLOAD *)command)->rate_cutoff;
      gEcuConfigPtr->accel_cut_off = ((DIGITAL_FILTER_PAYLOAD *)command)->accel_cutoff;
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_DIGITAL_FILTER;
    }
    break;   
  case SAE_J1939_GROUP_EXTENSION_ORIENTATION:
    if (((ORIENTATION_SETTING *)command)->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->rate_orien = ((ORIENTATION_SETTING *)command)->rate_orien;
      gEcuConfigPtr->accel_orien = ((ORIENTATION_SETTING *)command)->accel_orien;
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_ORIENTATION;
    }
    break;   
  case SAE_J1939_GROUP_EXTENSION_USER_BEHAVIOR:
    if (((USER_BEHAVIOR_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->restart_on_overrange = ((USER_BEHAVIOR_PAYLOAD *)command)->restart_on_overrange;
      gEcuConfigPtr->dynamic_motion = ((USER_BEHAVIOR_PAYLOAD *)command)->dynamic_motion;
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_USER_BEHAVIOR;
    }
    break;   
  case SAE_J1939_GROUP_EXTENSION_ANGLE_ALARM:
    if (((ANGLE_ALARM_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->roll_upper = ((ANGLE_ALARM_PAYLOAD *)command)->roll_upper;
      gEcuConfigPtr->roll_lower = ((ANGLE_ALARM_PAYLOAD *)command)->roll_lower;
      gEcuConfigPtr->pitch_upper = ((ANGLE_ALARM_PAYLOAD *)command)->pitch_upper;
      gEcuConfigPtr->pitch_lower = ((ANGLE_ALARM_PAYLOAD *)command)->pitch_lower;
      gEcuConfigPtr->roll_hysteresis = ((ANGLE_ALARM_PAYLOAD *)command)->roll_hysteresis;
      gEcuConfigPtr->pitch_hyseresis = ((ANGLE_ALARM_PAYLOAD *)command)->pitch_hyseresis;
      
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_ANGLE_ALARM;
    }
    break;   
  case SAE_J1939_GROUP_EXTENSION_CONE_ALARM:
    if (((CONE_ALARM_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->alarm_selector = ((CONE_ALARM_PAYLOAD *)command)->alarm_selector;
      gEcuConfigPtr->angle_limit = ((CONE_ALARM_PAYLOAD *)command)->angle_limit;
      gEcuConfigPtr->angle_hysteresis = ((CONE_ALARM_PAYLOAD *)command)->angle_hysteresis;
      
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_CONE_ALARM;
      
    }
    break;   
  case SAE_J1939_GROUP_EXTENSION_ACCELERATION_PARAM:
    if (((ACCELERATION_PARAM_PAYLOAD *)command)->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->acceleration_x = ((ACCELERATION_PARAM_PAYLOAD *)command)->x_acceleration;
      gEcuConfigPtr->acceleration_y = ((ACCELERATION_PARAM_PAYLOAD *)command)->y_acceleration;
      gEcuConfigPtr->acceleration_z = ((ACCELERATION_PARAM_PAYLOAD *)command)->z_acceleration;
           
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_ACCELERATION_PARAM;
    }
    break;
  case SAE_J1939_GROUP_EXTENSION_BANK0:
      gEcuConfigPtr->alg_reset_ps = ((BANK0_PS_PAYLOAD *)command)->alg_reset_ps;
      gEcuConfigPtr->save_cfg_ps = ((BANK0_PS_PAYLOAD *)command)->save_cfg_ps;
      gEcuConfigPtr->hardware_bit_ps = ((BANK0_PS_PAYLOAD *)command)->hardware_bit_ps;
      gEcuConfigPtr->software_bit_ps = ((BANK0_PS_PAYLOAD *)command)->software_bit_ps;
      gEcuConfigPtr->status_ps = ((BANK0_PS_PAYLOAD *)command)->status_ps;
      
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_GROUP_EXTENSION_BANK;
      break;
   case SAE_J1939_GROUP_EXTENSION_BANK1:
      gEcuConfigPtr->packet_rate_ps = ((BANK1_PS_PAYLOAD *)command)->packet_rate_ps;
      gEcuConfigPtr->packet_type_ps = ((BANK1_PS_PAYLOAD *)command)->packet_type_ps;
      gEcuConfigPtr->digital_filter_ps = ((BANK1_PS_PAYLOAD *)command)->digital_filter_ps;
      gEcuConfigPtr->orientation_ps = ((BANK1_PS_PAYLOAD *)command)->orientation_ps;
      gEcuConfigPtr->user_behavior_ps = ((BANK1_PS_PAYLOAD *)command)->user_behavior_ps;
      gEcuConfigPtr->angle_alarm_ps = ((BANK1_PS_PAYLOAD *)command)->angle_alarm_ps;
      gEcuConfigPtr->cone_alarm_ps = ((BANK1_PS_PAYLOAD *)command)->cone_alarm_ps;
      gEcuConfigPtr->acceleration_param_ps = ((BANK1_PS_PAYLOAD *)command)->acceleration_param_ps;
      
      gEcuConfigPtr->config_changed |= _ECU_CONFIG_GROUP_EXTENSION_BANK;
      break;   
  default:
      break;
  }          
  
  return;  
}

void process_config_set(struct sae_j1939_rx_desc *desc)
{
  SAE_J1939_IDENTIFIER_FIELD *ident;
  uint8_t pf_val, ps_val;
  uint8_t *command;
  
  if (desc == NULL)
    return;
  
  ident = &(desc->rx_identifier);
  if (ident == NULL)
    return;
  
  command = desc->rx_buffer.Data;
  if (command == NULL)
    return;
  
  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;
  
  if (pf_val != SAE_J1939_PDU_FORMAT_GLOBAL)
    return;
  
  ecu_command_set(command, ps_val);
  
  return;
  
}

void process_request(struct sae_j1939_rx_desc *desc)
{
  SAE_J1939_IDENTIFIER_FIELD *ident;
  uint8_t pf_val, req_pf_val, req_ps_val;
  uint8_t *command;
  
  if (desc == NULL)
    return;
  
  ident = &(desc->rx_identifier);
  if (ident == NULL)
    return;
  
  if (desc->rx_buffer.RTR || !desc->rx_buffer.IDE || (desc->rx_buffer.DLC != 3)) 
      return;
  
  command = desc->rx_buffer.Data;
  if (command == NULL)
    return;
  
  pf_val = ident->pdu_format;
  req_pf_val = command[1];
  req_ps_val = command[2];
  
  if (pf_val != SAE_J1939_PDU_FORMAT_REQUEST)
    return;
  
  switch(req_pf_val) {
    case SAE_J1939_PDU_FORMAT_SOFTWARE_VERSION:
        {
            memsic_j1939_send_software_version();
        }
        break;
    case SAE_J1939_PDU_FORMAT_ECU:
        {
            memsic_j1939_send_ecu_id();
        }
        break;
    case SAE_J1939_PDU_FORMAT_GLOBAL:
      if (req_ps_val == SAE_J1939_GROUP_EXTENSION_TEST_HARDWARE) {
        HARDWARE_TEST_PAYLOAD *hardware_bits;
        memsic_j1939_send_test_packet(MEMSIC_SAE_J1939_BUILTIN_HARDWARE, (void *)hardware_bits);
      }
      else if (req_ps_val == SAE_J1939_GROUP_EXTENSION_TEST_SOFTWARE) {
        SOFTWARE_TEST_PAYLOAD *software_bits;
        memsic_j1939_send_test_packet(MEMSIC_SAE_J1939_BUILTIN_SOFTWARE, (void *)software_bits);
      }
      else {
        STATUS_TEST_PAYLOAD *status_bits;
        memsic_j1939_send_test_packet(MEMSIC_SAE_J1939_BUILTIN_STATUS, (void *)status_bits);
      }
      break;
    case SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM:
       process_request_pg(desc);
      break;
    default:
      break;
  }
  
  return;
}

static void send_tilt_sensor_data()
{
  SLOPE_SENSOR_2 slope_data;
  AUGULAR_RATE   angle_data;
  ACCELERATION_SENSOR accel_data;
  float rate[3];
  int i;
  
  if( UcbGetSysType() > IMU_9DOF_SYS ) {
      if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_SLOPE_SENSOR) {
          slope_data.pitch = (uint32_t)((gKalmanFilter.eulerAngles[PITCH] * 57.296 + 250.00) * 32768);
          slope_data.roll = (uint32_t)((gKalmanFilter.eulerAngles[ROLL] * 57.296 + 250.00) * 32768);
          
          slope_data.pitch_compensation = 0;
          slope_data.pitch_merit = 0;
          slope_data.roll_compensation = 0;
          slope_data.roll_merit = 0;
          slope_data.measure_latency = 0;
  
          memsic_j1939_send_slope_sensor(&slope_data);  
      }
  
      if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_ACCELERATOR) {
        accel_data.acceleration_x = (uint16_t)((gKalmanFilter.correctedAccel_B[X_AXIS] + 320.00) * 100);
        accel_data.acceleration_y = (uint16_t)((gKalmanFilter.correctedAccel_B[Y_AXIS] + 320.00) * 100);
        accel_data.acceleration_z = (uint16_t)((gKalmanFilter.correctedAccel_B[Z_AXIS] + 320.00) * 100);
     }
  
     if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_ANGULAR_RATE) {
        rate[0] = gKalmanFilter.correctedRate_B[Y_AXIS] * 57.296;
        rate[1] = gKalmanFilter.correctedRate_B[X_AXIS] * 57.296;
        rate[2] = gKalmanFilter.correctedRate_B[Z_AXIS] * 57.296;
    
        
     }
  } else {
    if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_ACCELERATOR) {
        accel_data.acceleration_x = (uint16_t)((gAlgorithm.scaledSensors[XACCEL] + 320.00) * 100);
        accel_data.acceleration_y = (uint16_t)((gAlgorithm.scaledSensors[YACCEL] + 320.00) * 100);
        accel_data.acceleration_z = (uint16_t)((gAlgorithm.scaledSensors[ZACCEL] + 320.00) * 100);
     }
  
     if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_ANGULAR_RATE) {
        rate[0] = (float)(gAlgorithm.scaledSensors[YRATE] * 57.296);
        rate[1] = (float)(gAlgorithm.scaledSensors[XRATE] * 57.296);
        rate[2] = (float)(gAlgorithm.scaledSensors[ZRATE] * 57.296);
     }
  }
  
    
  accel_data.acceleration_x_merit = 0;
  accel_data.acceleration_y_merit = 0;
  accel_data.acceleration_z_merit = 0;
  accel_data.acceleration_transmission_rate = 0;
    
  memsic_j1939_send_acceleration(&accel_data);
  
  for (i = 0; i < 3; i++) {
    if (rate[i] < -250.00)
      rate[i] = -250.00;
      
    if (rate[i] > 250.00)
      rate[i] = 250.00;
  }
      
  angle_data.pitch_rate = (uint16_t)((rate[0] + 250.00) * 128);
  angle_data.roll_rate = (uint16_t)((rate[1] + 250.00) * 128);
  angle_data.yaw_rate = (uint16_t)((rate[2] + 250.00) * 128);
  
  angle_data.pitch_rate_merit = 0;
  angle_data.roll_rate_merit = 0;
  angle_data.yaw_rate_merit = 0;
    
  memsic_j1939_send_angular_rate(&angle_data);
  
  return;  
}

void ecu_process(void)
{
  struct sae_j1939_rx_desc *rx_desc = gEcu->curr_process_desc;
  struct sae_j1939_tx_desc *tx_desc;
  MEMSIC_J1939_PACKET_TYPE incoming_type;
  uint8_t result;
  ECU_ADDRESS_ENTRY ecu_entry;
  uint8_t bits[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  
  memcpy((void *)&ecu_entry.ecu_name, (void *)gEcu->name, 8);
  ecu_entry.address = *gEcu->addr;
  ecu_entry.status = _ECU_NORMAL;
  ecu_entry.category = _ECU_MASTER;
  
  if (rx_desc == NULL)
    return;
  
  while (rx_desc->rx_pkt_ready == DESC_OCCUPIED) {
    rx_desc->rx_identifier.control_bits.priority = (rx_desc->rx_buffer.ExtId >> 26) & 0x7;
    rx_desc->rx_identifier.control_bits.data_page = (rx_desc->rx_buffer.ExtId >> 24) & 0x1;
    rx_desc->rx_identifier.pdu_format = (rx_desc->rx_buffer.ExtId >> 16) & 0xff;
    rx_desc->rx_identifier.pdu_specific = (rx_desc->rx_buffer.ExtId >> 8) & 0xff;
    rx_desc->rx_identifier.source = rx_desc->rx_buffer.ExtId & 0xff;
    
    incoming_type = is_valid_j1939_rcv(rx_desc);
    if ((incoming_type == MEMSIC_J1939_IGNORE) ||
        (incoming_type == MEMSIC_J1939_INVALID_IDENTIFIER)) {
      rx_desc->rx_pkt_ready = DESC_IDLE;
      rx_desc = rx_desc->next;
      ERROR_STRING("invalid j1939 packet");
      ERROR_ENDLINE();
      continue;
    }
    
    switch (incoming_type) {
    case MEMSIC_J1939_ADDRESS_CLAIM:
      process_address_claim(rx_desc);
      break;
    case MEMSIC_J1939_REQUEST_PACKET:
      process_request(rx_desc);
      break; 
    default:
      process_config_set(rx_desc);
      break;
    }
    
    rx_desc->rx_pkt_ready = DESC_IDLE;
    rx_desc = rx_desc->next;
  }
  
  gEcu->curr_process_desc =  rx_desc;


  {
    ECU_ADDRESS_ENTRY target;
    char test_pkt[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    target.address = 0x87;
    
    gEcuConfigPtr->packet_type = MEMSIC_SAE_J1939_PACKET_SLOPE_SENSOR | MEMSIC_SAE_J1939_PACKET_ACCELERATOR | 
                                 MEMSIC_SAE_J1939_PACKET_ANGULAR_RATE;
    send_tilt_sensor_data();
    //memsic_j1939_send_user_behavior(&target, test_pkt);
  }
   
  tx_desc = gEcu->curr_tx_desc;
  while (tx_desc->tx_pkt_ready != DESC_IDLE) 
  {
    if (tx_desc->tx_pkt_ready == DESC_PENDING) {
      tx_desc = tx_desc->next;
      continue;
    }
      
    if ((result = gEcu->xmit(tx_desc)) != CAN_TxStatus_NoMailBox) {
      tx_desc->tx_pkt_ready = DESC_PENDING;
      tx_desc = tx_desc->next;
    }
    else
      return;
  }
  
  return;
}

void memsic_j1939_transmit_isr(void)
{
  struct sae_j1939_tx_desc *desc;
  
  desc = gEcu->curr_tx_desc;
  
  if ((desc == NULL) || (desc->next == NULL) || (desc->tx_pkt_ready == DESC_IDLE))
    return;
  
  desc->tx_pkt_ready = DESC_IDLE;
  
  if (desc->next->tx_pkt_ready == DESC_PENDING)
    gEcu->curr_tx_desc = gEcu->curr_tx_desc->next;
  
  return;
}

void memsic_j1939_receive_isr(void)
{
  struct sae_j1939_rx_desc *desc;
  
  desc = gEcu->curr_rx_desc;
  
  if ((desc == NULL) || (desc->next == NULL) || (desc->rx_pkt_ready == DESC_OCCUPIED))
    return;
  
  desc->rx_pkt_ready = DESC_OCCUPIED;
  
  if (desc->next->rx_pkt_ready == DESC_IDLE)
    gEcu->curr_rx_desc = gEcu->curr_rx_desc->next;
  
  return;
}
#endif
