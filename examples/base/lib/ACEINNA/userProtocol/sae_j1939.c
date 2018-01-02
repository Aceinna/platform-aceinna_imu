/** ***************************************************************************
 * @file sae_j1939.c the definitions of basic functions
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

#include "stm32f2xx.h"
#include "DLib_Product_string.h"
#include "timer.h"
#include "debug.h"
#include "xbowsp_init.h"
#include "can.h"

#include "sae_j1939.h"

ECU_INSTANCE gEcuInst;
EcuConfigurationStruct gEcuConfig;
ECU_ADDRESS_ENTRY addrMappingTable[SAE_J1939_MAX_TABLE_ENTRY];

ECU_ADDRESS_ENTRY *gAddrMapTblPtr = &addrMappingTable[0];
EcuConfigurationStruct *gEcuConfigPtr = &gEcuConfig;

struct sae_j1939_tx_desc ecu_tx_desc[SAE_J1939_MAX_TX_DESC];
struct sae_j1939_rx_desc ecu_rx_desc[SAE_J1939_MAX_RX_DESC];

#define SAE_J1939_TX_THRESHOLD                     16
#define SAE_J1939_MAX_IDLE_TIME                    1000000    //ms

static MEMSIC_ECU_ADDR ecu_pool[MEMSIC_ECU_ADDRESS_MAX];

uint8_t add_ecu_mapping_table(uint8_t addr, SAE_J1939_NAME_FIELD name) 
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for ( i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (addrTbl->status == _ECU_IDLE)
      break;
    addrTbl++;
  }
  
  if ( i == SAE_J1939_MAX_TABLE_ENTRY)
    return 0;
  
  addrTbl->ecu_name.words = name.words;
  addrTbl->address = addr;
  addrTbl->status = _ECU_NORMAL;
  addrTbl->category = _ECU_SLAVE;
  addrTbl->last_scan_time = 0;
  addrTbl->idle_time = 0;
  addrTbl->alive_time = 0;
  
  return 1;
}

uint8_t send_j1939_packet(struct sae_j1939_tx_desc *desc)
{
  CanTxMsg* TxMsg;
  uint8_t result = 0;
    
  TxMsg = &(desc->tx_buffer);
  
  result = CAN_Transmit(CAN1, TxMsg);
  
  result = CAN_TransmitStatus(CAN1, 0);
  
  return result;
  
}

void initialize_j1939_config(void)
{
   if (!(gEcuConfigPtr->ecu_name.words) || ( gEcuConfigPtr->ecu_name.bits.function != MEMSIC_SAE_J1939_FUNCTION)
      || (gEcuConfigPtr->ecu_name.bits.manufacture_code != MEMSIC_SAE_J1939_MANUFACTURE_CODE) 
      || !(gEcuConfigPtr->ecu_name.bits.identity_number)) 
  {
          gEcuInst.state = _ECU_INVALID_NAME;
          DEBUG_STRING("Invalid ECU device");
          ERROR_ENDLINE();
          return;
  }
 
  // set rate to 2Hz now, remove later
  gEcuConfigPtr->packet_rate = MEMSIC_SAE_J1939_PACKET_RATE_100;
    
  gEcuConfigPtr->config_changed = 0;
  gEcuConfigPtr->baudRate = _ECU_250K;
  
  if (!gEcuConfigPtr->alg_reset_ps)
      gEcuConfigPtr->alg_reset_ps = SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET;
  
  if (!gEcuConfigPtr->save_cfg_ps)
      gEcuConfigPtr->save_cfg_ps = SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION;
  
  if (!gEcuConfigPtr->hardware_bit_ps)
    gEcuConfigPtr->hardware_bit_ps = SAE_J1939_GROUP_EXTENSION_TEST_HARDWARE;
  
  if (!gEcuConfigPtr->software_bit_ps)
    gEcuConfigPtr->software_bit_ps = SAE_J1939_GROUP_EXTENSION_TEST_SOFTWARE;
  
  if (!gEcuConfigPtr->status_ps)
    gEcuConfigPtr->status_ps = SAE_J1939_GROUP_EXTENSION_TEST_STATUS;
  
  if (!gEcuConfigPtr->packet_rate_ps)
    gEcuConfigPtr->packet_rate_ps = SAE_J1939_GROUP_EXTENSION_PACKET_RATE;
  
  if (!gEcuConfigPtr->packet_type_ps)
    gEcuConfigPtr->packet_type_ps = SAE_J1939_GROUP_EXTENSION_PACKET_TYPE;
  
  if (!gEcuConfigPtr->digital_filter_ps)
    gEcuConfigPtr->digital_filter_ps = SAE_J1939_GROUP_EXTENSION_DIGITAL_FILTER;
  
  if (!gEcuConfigPtr->orientation_ps)
    gEcuConfigPtr->orientation_ps = SAE_J1939_GROUP_EXTENSION_ORIENTATION;
  
  if (!gEcuConfigPtr->user_behavior_ps)
    gEcuConfigPtr->user_behavior_ps = SAE_J1939_GROUP_EXTENSION_USER_BEHAVIOR;
  
  if (!gEcuConfigPtr->angle_alarm_ps)
    gEcuConfigPtr->angle_alarm_ps = SAE_J1939_GROUP_EXTENSION_ANGLE_ALARM;
  
  if (!gEcuConfigPtr->cone_alarm_ps)
    gEcuConfigPtr->cone_alarm_ps = SAE_J1939_GROUP_EXTENSION_CONE_ALARM;
  
  if (!gEcuConfigPtr->acceleration_param_ps)
    gEcuConfigPtr->acceleration_param_ps = SAE_J1939_GROUP_EXTENSION_ACCELERATION_PARAM;
  
  return;
}

void process_j1939_packet(struct sae_j1939_rx_desc * rx_desc)
{
  if (rx_desc->rx_pkt_ready != DESC_OCCUPIED)
    return;
  
   ecu_process();
  
  return;
}

void sae_j1939_initialize()
{
  int i;
  struct sae_j1939_tx_desc *tx_desc_ptr = &(ecu_tx_desc[0]);
  struct sae_j1939_rx_desc *rx_desc_ptr = &(ecu_rx_desc[0]);
  
   
  initialize_j1939_config();
  
  for ( i = 0; i < SAE_J1939_MAX_TX_DESC; i++) {
    memset((void *)tx_desc_ptr, '\0', sizeof(struct sae_j1939_tx_desc));
    if (i == SAE_J1939_MAX_TX_DESC - 1)
      tx_desc_ptr->next = &(ecu_tx_desc[0]);
    else {
      tx_desc_ptr->next = &(ecu_tx_desc[i]);
      tx_desc_ptr = tx_desc_ptr->next;
    }
  }
  
  
  for (i = 0; i < SAE_J1939_MAX_RX_DESC; i++) {
    memset((void *)rx_desc_ptr, '\0', sizeof(struct sae_j1939_rx_desc));
    if (i == SAE_J1939_MAX_RX_DESC - 1)
      rx_desc_ptr->next = &(ecu_rx_desc[0]);
    else {
      rx_desc_ptr->next = &(ecu_rx_desc[i]);
      rx_desc_ptr = rx_desc_ptr->next;
    }
  }
  
  for (i = 0; i < MEMSIC_ECU_ADDRESS_MAX; i++) {
    ecu_pool[i].status = _ECU_ADDR_AVAILABLE;
    ecu_pool[i].addr = i + 128;
  }
  
  initialize_mapping_table();
  
  gEcuInst.name = (SAE_J1939_NAME_FIELD *)&gEcuConfigPtr->ecu_name;
  gEcuInst.addr = &(gEcuConfig.address);
  gEcuInst.category = _ECU_MASTER;
  if (gEcuInst.addr)
    gEcuInst.state = _ECU_CHECK_ADDRESS;
  else
    gEcuInst.state = _ECU_WAIT_ADDRESS;
  gEcuInst.addrTbl = gAddrMapTblPtr;
  
  // FL hardcode address here
  
  gEcuConfig.address = gConfiguration.ecuAddress;
  gEcuInst.state = _ECU_READY;
  
  gEcuInst.curr_tx_desc = &(ecu_tx_desc[0]);
  gEcuInst.curr_rx_desc = &(ecu_rx_desc[0]);
  gEcuInst.curr_process_desc = &(ecu_rx_desc[0]);
  
  gEcuInst.init_table = initialize_mapping_table;
  gEcuInst.update_table = update_mapping_table;
  gEcuInst.add_entry = add_ecu_mapping_table;
  gEcuInst.del_entry = del_ecu_mapping_table;
  gEcuInst.xmit = send_j1939_packet;
  
#ifndef MEMSIC_CAN_HOST
  _CAN_Configure(memsic_j1939_transmit_isr, memsic_j1939_receive_isr);
#endif
  return ;
}

void initialize_mapping_table(void)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for (i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (!addrTbl->ecu_name.words)
        addrTbl->status = _ECU_IDLE;
    else if (!addrTbl->address)
        addrTbl->status = _ECU_EMPTY_ADDRESS;
    else
        addrTbl->status = _ECU_EXPIRED;
    
    addrTbl++;
  }
    
  return;
}

void build_request_pkt(struct sae_j1939_tx_desc * req_desc)
{
  req_desc->tx_pkt_type = SAE_J1939_REQUEST_PACKET;
  req_desc->tx_payload_len = SAE_J1939_REQUEST_LEN;
  req_desc->tx_identifier.control_bits.priority = SAE_J1939_REQUEST_PRIORITY;
  req_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_REQUEST;
  req_desc->tx_identifier.pdu_specific = SAE_J1939_PDU_FORMAT_GLOBAL;
  req_desc->tx_identifier.source = *(uint8_t *)gEcuInst.addr;
  req_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  return;
}

void send_address_claim(ECU_INSTANCE *ecu)
{
  struct sae_j1939_tx_desc *addr_claim_desc;
  ADDR_CLAIM_PG_PACKET     addr_claim_pg;
  uint8_t address = *ecu->addr;
  
  if (gEcuInst.state < _ECU_READY) 
    return;
  
  if (find_tx_desc(&addr_claim_desc) == 0)
    return;
  
  if (!address) {
    addr_claim_pg.addr_claim_pg_pgn.r = 0;
    addr_claim_pg.addr_claim_pg_pgn.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
    addr_claim_pg.addr_claim_pg_pgn.pdu_format = SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM;
    addr_claim_pg.addr_claim_pg_pgn.pdu_specific = SAE_J1939_GROUP_EXTENSION_ACK;
  
    build_request_pkt(addr_claim_desc);
    
    addr_claim_desc->tx_identifier.source = SAE_J1939_GROUP_EXTENSION_ADDR;
  
    addr_claim_desc->tx_buffer.Data[0] = addr_claim_pg.addr_claim_pg_pgn.r;
    addr_claim_desc->tx_buffer.Data[1] = addr_claim_pg.addr_claim_pg_pgn.pdu_format;
    addr_claim_desc->tx_buffer.Data[2] = addr_claim_pg.addr_claim_pg_pgn.pdu_specific;
      
  } else { 
      addr_claim_desc->tx_pkt_type = MEMSIC_J1939_ADDRESS_CLAIM;
      addr_claim_desc->tx_payload_len = SAE_J1939_PAYLOAD_MAX_LEN;
      addr_claim_desc->tx_identifier.control_bits.priority = SAE_J1939_REQUEST_PRIORITY;
      addr_claim_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM;
      addr_claim_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_ACK;
      addr_claim_desc->tx_identifier.source = *(uint8_t *)gEcuInst.addr;
      
      memcpy((void *)addr_claim_desc->tx_buffer.Data, (void *)(&(ecu->name->words)), SAE_J1939_PAYLOAD_MAX_LEN);
  }
  
  addr_claim_desc->tx_pkt_ready = DESC_OCCUPIED;
  
 
  addr_claim_desc->tx_buffer.ExtId = ((addr_claim_desc->tx_identifier.r << 24) |
                                      (addr_claim_desc->tx_identifier.pdu_format << 16) |
                                      (addr_claim_desc->tx_identifier.pdu_specific << 8) |
                                      (addr_claim_desc->tx_identifier.source));
  addr_claim_desc->tx_buffer.IDE = CAN_ID_EXT;
  addr_claim_desc->tx_buffer.RTR = CAN_RTR_Data;
  addr_claim_desc->tx_buffer.DLC = addr_claim_desc->tx_payload_len;
  
  return;
}

void scan_address_mapping_table(void)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for ( i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (addrTbl->status == _ECU_NORMAL) {
        addrTbl->idle_time += TimePassed(addrTbl->last_scan_time); 
        addrTbl->alive_time += TimePassed(addrTbl->last_scan_time) / 1000;
        addrTbl->last_scan_time = TimeNow();
        
        if (addrTbl->idle_time > SAE_J1939_MAX_IDLE_TIME) 
          addrTbl->status = _ECU_EXPIRED;
    }
    
    if (i >= SAE_J1939_TX_THRESHOLD)
      break;
    
    if ((addrTbl->status == _ECU_EMPTY_ADDRESS) || (addrTbl->status == _ECU_EXPIRED))   {
      ECU_INSTANCE target_ecu;
      
      target_ecu.name = &addrTbl->ecu_name;
      target_ecu.addr = &addrTbl->address;
      
      send_address_claim(&target_ecu);
    }
  }
  
  return;
}

uint8_t allocate_ecu_addr(void)
{
  int i;
  
  for (i = 0; i < MEMSIC_ECU_ADDRESS_MAX; i++) {
    if (ecu_pool[i].status == _ECU_ADDR_AVAILABLE) {
      ecu_pool[i].status = _ECU_ADDR_OCCUPIED;
      return ecu_pool[i].addr;
    }
  }
  
  return 247;
}



uint8_t del_ecu_mapping_table(ECU_ADDRESS_ENTRY *input)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for ( i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (addrTbl->ecu_name.words == input->ecu_name.words) {
      ecu_pool[addrTbl->address - 128].status = _ECU_ADDR_AVAILABLE;
      memset((void *)addrTbl, '\0', sizeof(ECU_ADDRESS_ENTRY));
      addrTbl->status = _ECU_IDLE;
      
      break;
    }
  }
  
  return 0;
}

void update_mapping_table(ECU_ADDRESS_ENTRY *entry)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
   
  for (i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (entry->ecu_name.words == addrTbl->ecu_name.words) {
      addrTbl->address = entry->address;
      addrTbl->status = entry->status;
      addrTbl->category = entry->category;
      if (entry->status == _ECU_NORMAL)
          addrTbl->idle_time = 0;
      
      return;
    }
    
    addrTbl++;
  }
  
  add_ecu_mapping_table(entry->address, entry->ecu_name);
 
  return;    
}

ECU_ADDRESS_ENTRY * find_remote_ecu(uint8_t addr, SAE_J1939_NAME_FIELD name)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for (i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (name.words == addrTbl->ecu_name.words) {
      if (addr != addrTbl->address)
        addrTbl->address = addr;
      return addrTbl;
    }
    
    addrTbl++;
  }
  
  return NULL;  
}

uint8_t find_tx_desc(struct sae_j1939_tx_desc **input)
{
  *input = gEcuInst.curr_tx_desc;
  
  while ((*input)->tx_pkt_ready == DESC_OCCUPIED) {
    *input =  (*input)->next;
    if (*input == gEcuInst.curr_tx_desc) {
      gEcuInst.state = _ECU_TX_OVERFLOW;
      return 0;
    }
  }
  
  return 1;
}

void build_set_pkt(struct sae_j1939_tx_desc * req_desc, MEMSIC_SAE_J1939_CONTROL ctrl_type, _ECU_CATEGORY category)
{
  if (category == _ECU_MASTER)
      req_desc->tx_pkt_type = SAE_J1939_SET_PACKET;
  else
      req_desc->tx_pkt_type = SAE_J1939_RESPONSE_PACKET;
  
  req_desc->tx_identifier.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
  req_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_GLOBAL;
  req_desc->tx_identifier.source = *(uint8_t *)gEcuInst.addr;
  req_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  switch (ctrl_type) {
      case MEMSIC_SAE_J1939_ALGORITHM_RESET:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_ALGO_RST_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->alg_reset_ps;
         break;
      case MEMSIC_SAE_J1939_CONFIG_SAVE:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_SAVE_CONFIG_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->save_cfg_ps;
         break;
      case MEMSIC_SAE_J1939_PACKET_TYPE:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_PACKET_TYPE_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->packet_type_ps;
         break;
      case MEMSIC_SAE_J1939_RATE_DIVIDER:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_PACKET_RATE_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->packet_rate_ps;
         break;
      case MEMSIC_SAE_J1939_DIGITAL_FILTER:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_DIGITAL_FILTER_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->digital_filter_ps;
         break;
      case MEMSIC_SAE_J1939_ORIENTATION:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_ORIENTATION_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->orientation_ps;
         break;    
      case MEMSIC_SAE_J1939_USER_BEHAVIOR:   
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_USER_BEHAVIOR_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->user_behavior_ps;
         break;
      case MEMSIC_SAE_J1939_ANGLE_ALARM:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_ANGLE_ALARM_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->angle_alarm_ps;
         break;
      case MEMSIC_SAE_J1939_CONE_ALARM:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_CONE_ALARM_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->cone_alarm_ps;
         break;
      case MEMSIC_SAE_J1939_ACCELERATION_PARAMETERS:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_ACCELERATION_PARAM_LEN;
         req_desc->tx_identifier.pdu_specific = gEcuConfigPtr->acceleration_param_ps;
         break;
      case MEMSIC_SAE_J1939_PS_BANK0:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_BANK0_LEN;
         req_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_BANK0;
         break;
      case MEMSIC_SAE_J1939_PS_BANK1:
         req_desc->tx_payload_len = MEMSIC_SAE_J1939_BANK1_LEN;
         req_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_BANK1;
         break;
  default:
         break;
  }
  
  req_desc->tx_buffer.ExtId = ((req_desc->tx_identifier.r << 24) |
                               (req_desc->tx_identifier.pdu_format << 16) |
                               (req_desc->tx_identifier.pdu_specific << 8) |
                               (req_desc->tx_identifier.source));
  req_desc->tx_buffer.IDE = CAN_ID_EXT;
  req_desc->tx_buffer.RTR = CAN_RTR_Data;
  req_desc->tx_buffer.DLC = req_desc->tx_payload_len;
  
  return;
}

void build_command_set(COMMAND_SET_PAYLOAD *command, ECU_ADDRESS_ENTRY *target, _ECU_CATEGORY category)
{
  if (category == _ECU_MASTER)
      command->request = MEMSIC_SAE_J1939_REQUEST;
  else
      command->request = MEMSIC_SAE_J1939_RESPONSE;
  
  command->dest_address = target->address;
  
  return;
}

uint8_t is_valid_pf(uint8_t pf_val)
{
  if ((pf_val == SAE_J1939_PDU_FORMAT_ACK) ||
      (pf_val == SAE_J1939_PDU_FORMAT_REQUEST) ||
      (pf_val == SAE_J1939_PDU_FORMAT_DATA) ||
      (pf_val == SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM) ||
      (pf_val == SAE_J1939_PDU_FORMAT_ECU) ||
      (pf_val == SAE_J1939_PDU_FORMAT_SOFTWARE_VERSION) ||
      (pf_val == SAE_J1939_PDU_FORMAT_GLOBAL))
      return 1;
       
   return 0;
}

void process_request_pg(struct sae_j1939_rx_desc *rx_desc)
{
     
  if (rx_desc == NULL)
    return;
  
  send_address_claim(&gEcuInst);
  
  return;  
}
   
void process_address_claim(struct sae_j1939_rx_desc *desc)
{
  uint8_t source_addr, result;
  SAE_J1939_NAME_FIELD ecu_name;
  SAE_J1939_IDENTIFIER_FIELD * ident;
  ECU_ADDRESS_ENTRY * remote_ecu;
  
  if (desc == NULL)
    return;
  
  ident = &desc->rx_identifier;
  if (ident == NULL)
    return;
  
  source_addr = ident->source;
  memcpy((void *)&ecu_name.words, (void *)desc->rx_buffer.Data, 8); 
  remote_ecu = find_remote_ecu(source_addr, ecu_name);
  if ((remote_ecu == NULL) && (source_addr != *gEcuInst.addr)) {
    if (!(result = add_ecu_mapping_table(source_addr, ecu_name))) {
      ERROR_STRING("\r\naddress table full.\r\n");
    }
  } else if (source_addr == *gEcuInst.addr) {
    if (remote_ecu->ecu_name.words > gEcuInst.name->words)
      send_address_claim(&gEcuInst);
    else 
      *gEcuInst.addr = allocate_ecu_addr();  
  }
  
  return;  
  
}

uint8_t is_valid_sae_j1939_identifier(SAE_J1939_IDENTIFIER_FIELD *identifier)
{
  if (identifier->control_bits.data_page) 
    return 0;
  
  if (!is_valid_pf(identifier->pdu_format))
    return 0;
  
  return 1;
}

MEMSIC_J1939_PACKET_TYPE is_valid_address_claim(SAE_J1939_IDENTIFIER_FIELD *ident)
{
  uint8_t pf_val, ps_val;
 
  if (ident == NULL)
    return MEMSIC_J1939_INVALID_IDENTIFIER;
    
  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM) &&
      (ps_val == SAE_J1939_GROUP_EXTENSION_ACK)) {
       return MEMSIC_J1939_ADDRESS_CLAIM;
  }
  
  return MEMSIC_J1939_IGNORE;
}

MEMSIC_J1939_PACKET_TYPE is_data_packet(SAE_J1939_IDENTIFIER_FIELD *ident)
{
  uint8_t pf_val, ps_val;
 
  
  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_DATA) &&
      ((ps_val == SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_ACCELERATION))) {
       return MEMSIC_J1939_DATA;
  }
  
  return MEMSIC_J1939_IGNORE;
}

MEMSIC_J1939_PACKET_TYPE is_valid_config_command(SAE_J1939_IDENTIFIER_FIELD *ident)
{
   uint8_t pf_val, ps_val;
   
   if (ident == NULL)
     return MEMSIC_J1939_INVALID_IDENTIFIER;
   
   pf_val = ident->pdu_format;
   ps_val = ident->pdu_specific;
   
   if ((pf_val == SAE_J1939_PDU_FORMAT_GLOBAL) &&
       ((ps_val >= SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET) &&
       (ps_val <= SAE_J1939_GROUP_EXTENSION_ACCELERATION_PARAM)) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_BANK0) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_BANK1)) {
         return MEMSIC_J1939_CONFIG;
   }
   
   return MEMSIC_J1939_IGNORE;
}
#endif //MEMSIC_CAN
