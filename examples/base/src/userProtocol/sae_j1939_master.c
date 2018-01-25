/** ***************************************************************************
 * @file sae_j1939_master.c the definitions of basic functions
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
#ifdef SAEJ1939
#include <stdio.h>
#include <stdlib.h>

#include "stm32f2xx.h"
#include "DLib_Product_string.h"
#include "timer.h"
#include "debug.h"

#include "sae_j1939.h"

ECU_INSTANCE * gMasterEcu = &gEcuInst;




uint8_t request_software_version(void)
{  
  struct sae_j1939_tx_desc *req_software_ver;
  VERSION_PACKET ver_pkt;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  ver_pkt.ver_pgn.r = 0;
  ver_pkt.ver_pgn.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  ver_pkt.ver_pgn.pdu_format = SAE_J1939_PDU_FORMAT_SOFTWARE_VERSION;
  ver_pkt.ver_pgn.pdu_specific = SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION;
  
  if (find_tx_desc(&req_software_ver) == 0)
    return 0;
  
  build_request_pkt(req_software_ver);
  
  req_software_ver->tx_buffer.ExtId = ((req_software_ver->tx_identifier.r << 24) |
                               (req_software_ver->tx_identifier.pdu_format << 16) |
                               (req_software_ver->tx_identifier.pdu_specific << 8) |
                               (req_software_ver->tx_identifier.source));
  req_software_ver->tx_buffer.IDE = CAN_ID_EXT;
  req_software_ver->tx_buffer.RTR = CAN_RTR_Data;
  req_software_ver->tx_buffer.DLC = req_software_ver->tx_payload_len;
  req_software_ver->tx_buffer.Data[0] = ver_pkt.ver_pgn.r;
  req_software_ver->tx_buffer.Data[1] = ver_pkt.ver_pgn.pdu_format;
  req_software_ver->tx_buffer.Data[2] = ver_pkt.ver_pgn.pdu_specific;
    
  gMasterEcu->state |= _ECU_WAIT_SOFTWARE_VER;
  
  return 1;
  
}

uint8_t request_ecu_id(void)
{
  struct sae_j1939_tx_desc *req_ecu_id;
  ECU_ID_PACKET ecu_id_pkt;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  ecu_id_pkt.ecu_id_pgn.r = 0;
  ecu_id_pkt.ecu_id_pgn.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  ecu_id_pkt.ecu_id_pgn.pdu_format = SAE_J1939_PDU_FORMAT_ECU;
  ecu_id_pkt.ecu_id_pgn.pdu_specific = SAE_J1939_GROUP_EXTENSION_ECU;
  
  if (find_tx_desc(&req_ecu_id) == 0)
    return 0;
  
  build_request_pkt(req_ecu_id);
  
  req_ecu_id->tx_buffer.ExtId = ((req_ecu_id->tx_identifier.r << 24) |
                                 (req_ecu_id->tx_identifier.pdu_format << 16) |
                                 (req_ecu_id->tx_identifier.pdu_specific << 8) |
                                 (req_ecu_id->tx_identifier.source));
  req_ecu_id->tx_buffer.IDE = CAN_ID_EXT;
  req_ecu_id->tx_buffer.RTR = CAN_RTR_Data;
  req_ecu_id->tx_buffer.DLC = req_ecu_id->tx_payload_len;
  req_ecu_id->tx_buffer.Data[0] = ecu_id_pkt.ecu_id_pgn.r;
  req_ecu_id->tx_buffer.Data[1] = ecu_id_pkt.ecu_id_pgn.pdu_format;
  req_ecu_id->tx_buffer.Data[2] = ecu_id_pkt.ecu_id_pgn.pdu_specific;
      
  gMasterEcu->state |= _ECU_WAIT_ID;
  
  return 1;
}

uint8_t alg_reset_request(ECU_ADDRESS_ENTRY *target)
{
  struct sae_j1939_tx_desc *req_alg_rst;
  COMMAND_SET_PAYLOAD *alg_rst_pkt;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_alg_rst) == 0)
    return 0;
  
  alg_rst_pkt = (COMMAND_SET_PAYLOAD *)req_alg_rst->tx_buffer.Data;
    
  build_set_pkt(req_alg_rst, MEMSIC_SAE_J1939_ALGORITHM_RESET, gMasterEcu->category);
  
  build_command_set(alg_rst_pkt, target, gMasterEcu->category);
  
  gMasterEcu->state |= _ECU_WAIT_ALG_RESET;
  
  return 1;
}

uint8_t save_config_request(ECU_ADDRESS_ENTRY *target)
{
  struct sae_j1939_tx_desc *req_save_cfg;
  COMMAND_SET_PAYLOAD * save_cfg_pkt;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_save_cfg) == 0)
    return 0;
  
  save_cfg_pkt = (COMMAND_SET_PAYLOAD *)req_save_cfg->tx_buffer.Data;
    
  build_set_pkt(req_save_cfg, MEMSIC_SAE_J1939_CONFIG_SAVE, gMasterEcu->category);
  
  build_command_set(save_cfg_pkt, target, gMasterEcu->category);
  
  gMasterEcu->state |= _ECU_WAIT_CONFIG_SAVE;
  return 1;
}

uint8_t request_builtin_test(uint8_t builtin_type)
{
  struct sae_j1939_tx_desc *req_builtin_test;
  SAE_J1939_IDENTIFIER_FIELD builtin_pgn;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  builtin_pgn.r = 0;
  builtin_pgn.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  builtin_pgn.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  if (builtin_type & MEMSIC_SAE_J1939_BUILTIN_HARDWARE) 
      builtin_pgn.pdu_specific = gEcuConfigPtr->hardware_bit_ps;
  else if (builtin_type & MEMSIC_SAE_J1939_BUILTIN_SOFTWARE) 
      builtin_pgn.pdu_specific = gEcuConfigPtr->software_bit_ps;
  else if (builtin_type & MEMSIC_SAE_J1939_BUILTIN_STATUS) 
      builtin_pgn.pdu_specific = gEcuConfigPtr->status_ps;
  
  if (find_tx_desc(&req_builtin_test) == 0)
    return 0;
  
  build_request_pkt(req_builtin_test);
  
  req_builtin_test->tx_buffer.ExtId = ((req_builtin_test->tx_identifier.r << 24) |
                                       (req_builtin_test->tx_identifier.pdu_format << 16) |
                                       (req_builtin_test->tx_identifier.pdu_specific << 8) |
                                       (req_builtin_test->tx_identifier.source));
  req_builtin_test->tx_buffer.IDE = CAN_ID_EXT;
  req_builtin_test->tx_buffer.RTR = CAN_RTR_Data;
  req_builtin_test->tx_buffer.DLC = req_builtin_test->tx_payload_len;
  req_builtin_test->tx_buffer.Data[0] = builtin_pgn.r;
  req_builtin_test->tx_buffer.Data[1] = builtin_pgn.pdu_format;
  req_builtin_test->tx_buffer.Data[2] = builtin_pgn.pdu_specific;
  
  gMasterEcu->state |= _ECU_WAIT_BUILTIN_TEST;
    
  return 1;
}

uint8_t packet_rate_request(ECU_ADDRESS_ENTRY *target, uint8_t odr)
{
  struct sae_j1939_tx_desc *req_packet_rate;
  RATE_CONFIG_PAYLOAD * packet_rate_cfg;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_packet_rate) == 0)
    return 0;
  
  packet_rate_cfg = (RATE_CONFIG_PAYLOAD *)req_packet_rate->tx_buffer.Data;
    
  build_set_pkt(req_packet_rate, MEMSIC_SAE_J1939_RATE_DIVIDER, gMasterEcu->category);
  
  packet_rate_cfg->dest_address = target->address;
  packet_rate_cfg->odr = odr;  
  
  return 1;
}

uint8_t packet_type_request(ECU_ADDRESS_ENTRY *target, uint8_t packet_type)
{
  struct sae_j1939_tx_desc *req_packet_type;
  PACKET_TYPE_PAYLOAD * packet_type_cfg;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_packet_type) == 0)
    return 0;
  
  packet_type_cfg = (PACKET_TYPE_PAYLOAD *)req_packet_type->tx_buffer.Data;
  packet_type_cfg->type_bits.r = packet_type;
    
  build_set_pkt(req_packet_type, MEMSIC_SAE_J1939_PACKET_TYPE, gMasterEcu->category);
  
  packet_type_cfg->dest_address = target->address;
  packet_type_cfg->type_bits.r &= MEMSIC_SAE_J1939_TYPE_MASK;  
  
  return 1;
}

uint8_t digital_filter_request(ECU_ADDRESS_ENTRY *target, uint8_t rate_freq, uint8_t accel_freq)
{
  struct sae_j1939_tx_desc *req_digital_filter;
  DIGITAL_FILTER_PAYLOAD * digital_filter_cfg;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_digital_filter) == 0)
    return 0;
  
  digital_filter_cfg = (DIGITAL_FILTER_PAYLOAD *)req_digital_filter->tx_buffer.Data;
    
  build_set_pkt(req_digital_filter, MEMSIC_SAE_J1939_DIGITAL_FILTER, gMasterEcu->category);
  
  digital_filter_cfg->dest_address = target->address;
  digital_filter_cfg->rate_cutoff = rate_freq;
  digital_filter_cfg->accel_cutoff = accel_freq;  
  
  return 1;
}
  
uint8_t orientation_request(ECU_ADDRESS_ENTRY *target, uint16_t orien)
{
  struct sae_j1939_tx_desc *req_orien;
  ORIENTATION_SETTING * orien_cfg;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_orien) == 0)
    return 0;
  
  orien_cfg = (ORIENTATION_SETTING *)req_orien->tx_buffer.Data;
    
  build_set_pkt(req_orien, MEMSIC_SAE_J1939_ORIENTATION, gMasterEcu->category);
  
  orien_cfg->dest_address = target->address;
  *(uint16_t *)orien_cfg->orien_bits = orien;
   
  return 1;
}


uint8_t user_behav_request(ECU_ADDRESS_ENTRY *target, uint8_t *behavior)
{
  struct sae_j1939_tx_desc *req_behav;
  USER_BEHAVIOR_PAYLOAD * user_behav_cfg;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_behav) == 0)
    return 0;
  
  user_behav_cfg = (USER_BEHAVIOR_PAYLOAD *)req_behav->tx_buffer.Data;
    
  build_set_pkt(req_behav, MEMSIC_SAE_J1939_USER_BEHAVIOR, gMasterEcu->category);
  
  user_behav_cfg->dest_address = target->address;
  user_behav_cfg->restart_on_overrange = behavior[0];
  user_behav_cfg->dynamic_motion = behavior[1];
    
  return 1;
}

uint8_t angle_alarm_request(ECU_ADDRESS_ENTRY *target, ANGLE_ALARM_PAYLOAD *angular_alarm)
{
  struct sae_j1939_tx_desc *req_angle_alarm;
  ANGLE_ALARM_PAYLOAD * angle_alarm_cfg;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_angle_alarm) == 0)
    return 0;
  
  angle_alarm_cfg = (ANGLE_ALARM_PAYLOAD *)req_angle_alarm->tx_buffer.Data;
    
  build_set_pkt(req_angle_alarm, MEMSIC_SAE_J1939_ANGLE_ALARM, gMasterEcu->category);
  
  angle_alarm_cfg->dest_address = target->address;
  angle_alarm_cfg->roll_upper = angular_alarm->roll_upper;
  angle_alarm_cfg->roll_lower = angular_alarm->roll_lower;
  angle_alarm_cfg->pitch_upper = angular_alarm->pitch_upper;
  angle_alarm_cfg->pitch_lower = angular_alarm->pitch_lower;
  angle_alarm_cfg->roll_hysteresis = angular_alarm->roll_hysteresis;
  angle_alarm_cfg->pitch_hyseresis = angular_alarm->pitch_hyseresis;
    
  return 1;
}

uint8_t cone_alarm_request(ECU_ADDRESS_ENTRY *target, CONE_ALARM_PAYLOAD *cone_alarm)
{
  struct sae_j1939_tx_desc *req_cone_alarm;
  CONE_ALARM_PAYLOAD * cone_alarm_cfg;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_cone_alarm) == 0)
    return 0;
  
  cone_alarm_cfg = (CONE_ALARM_PAYLOAD *)req_cone_alarm->tx_buffer.Data;
    
  build_set_pkt(req_cone_alarm, MEMSIC_SAE_J1939_CONE_ALARM, gMasterEcu->category);
  
  cone_alarm_cfg->dest_address = target->address;
  cone_alarm_cfg->alarm_selector = cone_alarm->alarm_selector;
  cone_alarm_cfg->angle_limit = cone_alarm->angle_limit;
  cone_alarm_cfg->angle_hysteresis = cone_alarm->angle_hysteresis;
    
  return 1;
}


uint8_t acceleration_param_request(ECU_ADDRESS_ENTRY *target, ACCELERATION_PARAM_PAYLOAD *accel_param)
{
  struct sae_j1939_tx_desc *req_accel_param;
  ACCELERATION_PARAM_PAYLOAD * accel_param_cfg;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_accel_param) == 0)
    return 0;
  
  accel_param_cfg = (ACCELERATION_PARAM_PAYLOAD *)req_accel_param->tx_buffer.Data;
    
  build_set_pkt(req_accel_param, MEMSIC_SAE_J1939_ACCELERATION_PARAMETERS, gMasterEcu->category);
  
  accel_param_cfg->dest_address = target->address;
  accel_param_cfg->x_acceleration = accel_param->x_acceleration;
  accel_param_cfg->y_acceleration = accel_param->y_acceleration;
  accel_param_cfg->z_acceleration = accel_param->z_acceleration;
    
  return 1;
}

uint8_t request_bank0(void)
{  
  struct sae_j1939_tx_desc *req_bank0;
  BANK0_PS_PAYLOAD bank0_ps;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  bank0_ps.bank0_pgn.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  bank0_ps.bank0_pgn.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  bank0_ps.bank0_pgn.pdu_specific = SAE_J1939_GROUP_EXTENSION_BANK0;
  
  if (find_tx_desc(&req_bank0) == 0)
    return 0;
  
  build_request_pkt(req_bank0);
  
  req_bank0->tx_buffer.ExtId = ((req_bank0->tx_identifier.r << 24) |
                                (req_bank0->tx_identifier.pdu_format << 16) |
                                (req_bank0->tx_identifier.pdu_specific << 8) |
                                (req_bank0->tx_identifier.source));
  req_bank0->tx_buffer.IDE = CAN_ID_EXT;
  req_bank0->tx_buffer.RTR = CAN_RTR_Data;
  req_bank0->tx_buffer.DLC = req_bank0->tx_payload_len;
  req_bank0->tx_buffer.Data[0] = bank0_ps.bank0_pgn.r;
  req_bank0->tx_buffer.Data[1] = bank0_ps.bank0_pgn.pdu_format;
  req_bank0->tx_buffer.Data[2] = bank0_ps.bank0_pgn.pdu_specific;
  
  return 1;
  
}

uint8_t request_bank1(void)
{  
  struct sae_j1939_tx_desc *req_bank1;
  BANK1_PS_PAYLOAD bank1_ps;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
 
  bank1_ps.bank1_pgn.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  bank1_ps.bank1_pgn.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  bank1_ps.bank1_pgn.pdu_specific = SAE_J1939_GROUP_EXTENSION_BANK1;
  
  if (find_tx_desc(&req_bank1) == 0)
    return 0;
  
  build_request_pkt(req_bank1);
  req_bank1->tx_buffer.ExtId = ((req_bank1->tx_identifier.r << 24) |
                                (req_bank1->tx_identifier.pdu_format << 16) |
                                (req_bank1->tx_identifier.pdu_specific << 8) |
                                (req_bank1->tx_identifier.source));
  req_bank1->tx_buffer.IDE = CAN_ID_EXT;
  req_bank1->tx_buffer.RTR = CAN_RTR_Data;
  req_bank1->tx_buffer.DLC = req_bank1->tx_payload_len;
  req_bank1->tx_buffer.Data[0] = bank1_ps.bank1_pgn.r;
  req_bank1->tx_buffer.Data[1] = bank1_ps.bank1_pgn.pdu_format;
  req_bank1->tx_buffer.Data[2] = bank1_ps.bank1_pgn.pdu_specific;
  
  return 1;
  
}

uint8_t bank0_request(ECU_ADDRESS_ENTRY *target, uint8_t *ps_num)
{
  struct sae_j1939_tx_desc *req_bank;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_bank) == 0)
    return 0;
  
  build_set_pkt(req_bank, MEMSIC_SAE_J1939_PS_BANK0, gMasterEcu->category);
  
  
  memcpy((void *)req_bank->tx_buffer.Data, (void *)ps_num, 8);
     
  return 1;
  
}

uint8_t bank1_request(ECU_ADDRESS_ENTRY *target, uint8_t *ps_num)
{
  struct sae_j1939_tx_desc *req_bank;
  
  if (gMasterEcu->state < _ECU_READY) 
    return 0;
  
  if (find_tx_desc(&req_bank) == 0)
    return 0;
  
  build_set_pkt(req_bank, MEMSIC_SAE_J1939_PS_BANK1, gMasterEcu->category);
  
  
  memcpy((void *)req_bank->tx_buffer.Data, (void *)ps_num, 8);
     
  return 1;
}

MEMSIC_J1939_PACKET_TYPE is_valid_software_version(SAE_J1939_IDENTIFIER_FIELD  *ident)
{
  uint8_t pf_val, ps_val; 
  
  if (ident == NULL)
    return MEMSIC_J1939_INVALID_IDENTIFIER;
    
  
  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;
  
  if (!(gMasterEcu->state & _ECU_WAIT_SOFTWARE_VER))
    return MEMSIC_J1939_IGNORE;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_SOFTWARE_VERSION) &&
      (ps_val == SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION)) {
      gMasterEcu->state &= ~_ECU_WAIT_SOFTWARE_VER;
      return MEMSIC_J1939_SOFTWARE_VERSION;
  }
  
  return MEMSIC_J1939_IGNORE;
}

MEMSIC_J1939_PACKET_TYPE is_valid_ecu_id(SAE_J1939_IDENTIFIER_FIELD  *ident)
{
  uint8_t pf_val, ps_val;
  
  if (ident == NULL)
    return MEMSIC_J1939_INVALID_IDENTIFIER;
    
  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;
  
  if (!(gMasterEcu->state & _ECU_WAIT_ID))
    return MEMSIC_J1939_IGNORE;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_ECU) &&
      (ps_val == SAE_J1939_GROUP_EXTENSION_ECU)) 
  {
       gMasterEcu->state &= ~_ECU_WAIT_ID;  
       return MEMSIC_J1939_ECU_ID;
  }
  
  return MEMSIC_J1939_IGNORE;
}

MEMSIC_J1939_PACKET_TYPE is_valid_alg_rst(struct sae_j1939_rx_desc *rx_desc)
{
  uint8_t pf_val, ps_val;
 
  if (rx_desc == NULL)
    return MEMSIC_J1939_INVALID_IDENTIFIER;
    
  pf_val = rx_desc->rx_identifier.pdu_format;
  ps_val = rx_desc->rx_identifier.pdu_specific;
  
  if (!(gMasterEcu->state &  _ECU_WAIT_ALG_RESET))
    return MEMSIC_J1939_IGNORE;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_GLOBAL) &&
      (ps_val == SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET) &&
      (rx_desc->rx_buffer.Data[0] == MEMSIC_SAE_J1939_RESPONSE) &&
      (rx_desc->rx_buffer.Data[1] == *(uint8_t *)gMasterEcu->addr)) {
       gMasterEcu->state &= ~_ECU_WAIT_ALG_RESET;  
       return MEMSIC_J1939_ALG_RST;
  }
  
  return MEMSIC_J1939_IGNORE;
}

MEMSIC_J1939_PACKET_TYPE is_valid_config_save(struct sae_j1939_rx_desc *rx_desc)
{
  uint8_t pf_val, ps_val;
 
  if (rx_desc == NULL)
    return MEMSIC_J1939_INVALID_IDENTIFIER;
    
  pf_val = rx_desc->rx_identifier.pdu_format;
  ps_val = rx_desc->rx_identifier.pdu_specific;
  
  if (!(gMasterEcu->state &  _ECU_WAIT_CONFIG_SAVE))
    return MEMSIC_J1939_IGNORE;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_GLOBAL) &&
      (ps_val == SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION) &&
      (rx_desc->rx_buffer.Data[0] == MEMSIC_SAE_J1939_RESPONSE) &&
      (rx_desc->rx_buffer.Data[1] == *(uint8_t *)gMasterEcu->addr)) {
       gMasterEcu->state &= ~_ECU_WAIT_CONFIG_SAVE;  
       return MEMSIC_J1939_CFG_SAVE;
  }
  
  return MEMSIC_J1939_IGNORE;
}

MEMSIC_J1939_PACKET_TYPE is_valid_built_in_test(struct sae_j1939_rx_desc *rx_desc)
{
  uint8_t pf_val, ps_val;
 
  if (rx_desc == NULL)
    return MEMSIC_J1939_INVALID_IDENTIFIER;
    
  pf_val = rx_desc->rx_identifier.pdu_format;
  ps_val = rx_desc->rx_identifier.pdu_specific;
  
  if (!(gMasterEcu->state &  _ECU_WAIT_BUILTIN_TEST))
    return MEMSIC_J1939_IGNORE;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_GLOBAL) &&
      ((ps_val == SAE_J1939_GROUP_EXTENSION_TEST_HARDWARE) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_TEST_SOFTWARE) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_TEST_STATUS))) {
       gMasterEcu->state &= ~_ECU_WAIT_BUILTIN_TEST;  
       return MEMSIC_J1939_BUILTIN_TEST;
  }
  
  return MEMSIC_J1939_IGNORE;
}

  
 
MEMSIC_J1939_PACKET_TYPE is_valid_request_pg(struct sae_j1939_rx_desc *rx_desc)
{
  uint8_t pf_val, ps_val;
 
  if (rx_desc == NULL)
    return MEMSIC_J1939_INVALID_IDENTIFIER;
    
  pf_val = rx_desc->rx_identifier.pdu_format;
  ps_val = rx_desc->rx_identifier.pdu_specific;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_REQUEST) &&
      (ps_val == SAE_J1939_GROUP_EXTENSION_ACK) &&
      (rx_desc->rx_buffer.Data[0] == SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM)) {
       return MEMSIC_J1939_REQUEST_PACKET;
  }
  
  return MEMSIC_J1939_IGNORE;
}



MEMSIC_J1939_PACKET_TYPE is_valid_j1939_master_recv(struct sae_j1939_rx_desc *rx_desc)
{
  SAE_J1939_IDENTIFIER_FIELD *ident = &(rx_desc->rx_identifier);
  MEMSIC_J1939_PACKET_TYPE result;
  
  if (!is_valid_sae_j1939_identifier(ident))
    return MEMSIC_J1939_INVALID_IDENTIFIER;
  
  result = is_data_packet(ident);
  if (result == MEMSIC_J1939_DATA)
     goto end_sae_j1939_packet;
  
  result = is_valid_address_claim(ident);
  if (result == MEMSIC_J1939_ADDRESS_CLAIM)
     goto end_sae_j1939_packet;
  
  result = is_valid_request_pg(rx_desc);
  if (result == MEMSIC_J1939_REQUEST_PACKET)
     goto end_sae_j1939_packet;
  
  result = is_valid_config_save(rx_desc);
  if (result == MEMSIC_J1939_CFG_SAVE)
    goto end_sae_j1939_packet;
  
 
  result = is_valid_alg_rst(rx_desc);
  if (result == MEMSIC_J1939_ALG_RST)
    goto end_sae_j1939_packet;
  
  result = is_valid_ecu_id(ident);
  if (result == MEMSIC_J1939_ECU_ID)
    goto end_sae_j1939_packet;
  
  result = is_valid_software_version(ident);
  if (result == MEMSIC_J1939_SOFTWARE_VERSION)
    goto end_sae_j1939_packet;
  

  
end_sae_j1939_packet:
  
  return result;        
}

void process_sensor_data(struct sae_j1939_rx_desc *rx_desc)
{
}

void process_config_save(struct sae_j1939_rx_desc *rx_desc)
{
}

void process_builtin_test(struct sae_j1939_rx_desc *rx_desc)
{
}

void process_alg_rst(struct sae_j1939_rx_desc *rx_desc)
{
}

void process_ecu_id(struct sae_j1939_rx_desc *rx_desc)
{
}

void process_software_ver(struct sae_j1939_rx_desc *rx_desc)
{
}
                         
void ecu_master_process(void)
{
  struct sae_j1939_rx_desc *rx_desc;
  MEMSIC_J1939_PACKET_TYPE incoming_type;
  
  rx_desc = gMasterEcu->curr_process_desc;
  if (rx_desc == NULL)
    return;
  
  while (rx_desc->rx_pkt_ready == DESC_OCCUPIED) {
    incoming_type = is_valid_j1939_master_recv(rx_desc);
    if ((incoming_type == MEMSIC_J1939_IGNORE) ||
        (incoming_type == MEMSIC_J1939_INVALID_IDENTIFIER)) {
      rx_desc->rx_pkt_ready = DESC_IDLE;
      rx_desc = rx_desc->next;
      ERROR_STRING("invalid j1939 packet");
      ERROR_ENDLINE();
      continue;
    }
    
    switch (incoming_type) {
    case MEMSIC_J1939_DATA:
      process_sensor_data(rx_desc);
      break;
    case MEMSIC_J1939_CFG_SAVE:
      process_config_save(rx_desc);
      break;
    case MEMSIC_J1939_BUILTIN_TEST:
      process_builtin_test(rx_desc);
      break;
    case MEMSIC_J1939_ALG_RST:
      process_alg_rst(rx_desc);
      break;
    case MEMSIC_J1939_ECU_ID:
      process_ecu_id(rx_desc);
      break;
    case MEMSIC_J1939_SOFTWARE_VERSION:
      process_software_ver(rx_desc);
      break;
    case MEMSIC_J1939_ADDRESS_CLAIM:
      process_address_claim(rx_desc);
      break;
    case MEMSIC_J1939_REQUEST_PACKET:
      process_request_pg(rx_desc);
      break; 
    default:
      break;
    }
    
    rx_desc->rx_pkt_ready = DESC_IDLE;
    rx_desc = rx_desc->next;
  }
  
  gMasterEcu->curr_process_desc =  rx_desc;
  
  if (gMasterEcu->curr_tx_desc->tx_pkt_ready == DESC_OCCUPIED)
  {
    // apply to DMA and disable interrupt
  }
  
  return;
}

void ecu_host_test(uint8_t test_type)
{
  ECU_ADDRESS_ENTRY ecu_entry;
  uint16_t orien = 0x008c;
  uint8_t alarm[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  ANGLE_ALARM_PAYLOAD *angle_alarm = (ANGLE_ALARM_PAYLOAD *)alarm;
  CONE_ALARM_PAYLOAD  *cone_alarm = (CONE_ALARM_PAYLOAD  *)alarm;
  ACCELERATION_PARAM_PAYLOAD *accel_param = (ACCELERATION_PARAM_PAYLOAD *)alarm;
  struct sae_j1939_tx_desc *tx_desc;
  uint8_t result;
  static uint32_t host_test_counter;
  
  memcpy((void *)&ecu_entry.ecu_name, (void *)gMasterEcu->name, 8);
  ecu_entry.address = *gMasterEcu->addr;
  ecu_entry.status = _ECU_NORMAL;
  ecu_entry.category = _ECU_MASTER;

  host_test_counter++;
  if (host_test_counter > 20)
    host_test_counter = 0;
 
  switch (host_test_counter) {
  case 1:
    request_software_version();
    break;
  case 2:
    request_ecu_id();
    break;
  case 3:
    alg_reset_request(&ecu_entry);
    break;
  case 4:
    save_config_request(&ecu_entry);
    break;
  case 5:
    //request_builtin_test(MEMSIC_SAE_J1939_BUILTIN_HARDWARE);
    break;
  case 6: 
    //request_builtin_test(MEMSIC_SAE_J1939_BUILTIN_SOFTWARE);
    break;
  case 7:
    //request_builtin_test(MEMSIC_SAE_J1939_BUILTIN_STATUS);
    break;
  case 8:
    packet_rate_request(&ecu_entry, 2);
    break;
  case 9: 
    packet_type_request(&ecu_entry, 1);
    break;
  case 10:
    digital_filter_request(&ecu_entry, 5, 10);
    break;
  case 11:
    orientation_request(&ecu_entry, orien);
    break;
  case 12:
    //user_behav_request(&ecu_entry, orien);
    break;
  case 13:
    //angle_alarm_request(&ecu_entry, angle_alarm);
    break;
  case 14:
    //cone_alarm_request(&ecu_entry, cone_alarm);
    break;
  case 15:
    //acceleration_param_request(&ecu_entry, accel_param); 
    break;
  case 16:
    request_bank0();
    break;
  case 17:
    request_bank1();
    break;
  case 18:
    bank0_request(&ecu_entry, alarm);
    bank1_request(&ecu_entry, alarm);
    break;
  case 19:
  case 20:
//    if (host_test_counter % 2)
//      *gMasterEcu->addr  = 0;
//    else
//      *gMasterEcu->addr = 0x85;
//    
//    send_address_claim(gMasterEcu);
    break;
  default:
    break;
  }
  
  tx_desc = gMasterEcu->curr_tx_desc;
  while (tx_desc->tx_pkt_ready != DESC_IDLE) 
  {
    if (tx_desc->tx_pkt_ready == DESC_PENDING) {
      tx_desc = tx_desc->next;
      continue;
    }
      
    if ((result = gMasterEcu->xmit(tx_desc)) != CAN_TxStatus_NoMailBox) {
      tx_desc->tx_pkt_ready = DESC_PENDING;
      tx_desc = tx_desc->next;
    }
    else
      return;
  }
  
  return;
    
}
#endif