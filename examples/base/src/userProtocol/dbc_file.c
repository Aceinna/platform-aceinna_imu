/** ***************************************************************************
 * @file   dbc_file.c the definitions of DBC functions
 * @Author Feng
 * @date   Dec 2017
 * @brief  Copyright (c) 2017, 2018 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifdef DBC_FILE
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
#include "xbowsp_algorithm.h"
#include "..\math\qmath.h"

#include "can.h"
#include "sae_j1939.h"
#include "dbc_file.h"

static uint8_t dbc_rate_cnt = 0, dbc_xl_cnt = 0;
static struct dbc_tx_desc dbc_tx_desc[DBC_MAX_TX_DESC];
static struct dbc_tx_desc *curr_tx_desc;

static uint8_t checksum (uint8_t *ptr, size_t sz) {
    uint8_t chk = 0;
    while (sz-- != 0)
        chk -= *ptr++;
    return chk;
}



static uint8_t send_dbc_packet(struct dbc_tx_desc *desc)
{
  CanTxMsg* TxMsg;
  uint8_t result = 0;
    
  TxMsg = &(desc->tx_buffer);
  
  result = CAN_Transmit(CAN1, TxMsg);
  
  result = CAN_TransmitStatus(CAN1, 0);
  
  return result;
  
}

void dbc_initialize()
{
  int i;
  struct dbc_tx_desc *tx_desc_ptr = &(dbc_tx_desc[0]);
    
  for ( i = 0; i < DBC_MAX_TX_DESC; i++) {
    memset((void *)tx_desc_ptr, '\0', sizeof(struct dbc_tx_desc));
    if (i == DBC_MAX_TX_DESC - 1)
      tx_desc_ptr->next = &(dbc_tx_desc[0]);
    else {
      tx_desc_ptr->next = &(dbc_tx_desc[i]);
      tx_desc_ptr = tx_desc_ptr->next;
    }
  }  
  
  curr_tx_desc = &(dbc_tx_desc[0]);
  
  _CAN_Configure(dbc_transmit_isr, NULL);

  return ;
}

static uint8_t find_dbc_desc(struct dbc_tx_desc **input)
{
  *input = curr_tx_desc;
  
  while ((*input)->tx_pkt_ready >= DESC_OCCUPIED) {
    *input =  (*input)->next;
    if (*input == curr_tx_desc) {
      return 0;
    }
  }
  
  return 1;
}

static uint8_t dbc_send_rate(void)
{
  struct dbc_tx_desc *dbc_desc;
  DBC_SG_PAYLOAD buffer;
  uint16_t rate[3];
  uint8_t csum_buf[10];
  
  if (find_dbc_desc(&dbc_desc) == 0)
    return 0;
   
  dbc_desc->tx_payload_len = DBC_PAYLOAD_LEN;
  dbc_desc->tx_identifier.b.can_id = ACEINNA_DBC_MESSAGE_RATE_ID;
  
  dbc_desc->tx_pkt_ready = DESC_OCCUPIED;
  dbc_rate_cnt++;
  
  dbc_desc->tx_buffer.StdId = dbc_desc->tx_identifier.r;
  dbc_desc->tx_buffer.IDE = CAN_ID_STD;
  dbc_desc->tx_buffer.RTR = CAN_RTR_Data;
  dbc_desc->tx_buffer.DLC = dbc_desc->tx_payload_len;
  
  if( UcbGetSysType() > IMU_9DOF_SYS ) {
//      if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_SLOPE_SENSOR) {
//          slope_data.pitch = (uint32_t)((gKalmanFilter.eulerAngles[PITCH] * 57.296 + 250.00) * 32768);
//          slope_data.roll = (uint32_t)((gKalmanFilter.eulerAngles[ROLL] * 57.296 + 250.00) * 32768);
//          
//          slope_data.pitch_compensation = 0;
//          slope_data.pitch_merit = 0;
//          slope_data.roll_compensation = 0;
//          slope_data.roll_merit = 0;
//          slope_data.measure_latency = 0;
//  
//          memsic_j1939_send_slope_sensor(&slope_data);  
//      }
//  
//      if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_ACCELERATOR) {
//        accel_data.acceleration_x = (uint16_t)(((gKalmanFilter.correctedAccel_B[X_AXIS] * GRAVITY) + 320.00) * 100);
//        accel_data.acceleration_y = (uint16_t)(((gKalmanFilter.correctedAccel_B[Y_AXIS] * GRAVITY) + 320.00) * 100);
//        accel_data.acceleration_z = (uint16_t)(((gKalmanFilter.correctedAccel_B[Z_AXIS] * GRAVITY) + 320.00) * 100);
//     }
//  
//     if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_ANGULAR_RATE) {
//        rate[0] = gKalmanFilter.correctedRate_B[Y_AXIS] * 57.296;
//        rate[1] = gKalmanFilter.correctedRate_B[X_AXIS] * 57.296;
//        rate[2] = gKalmanFilter.correctedRate_B[Z_AXIS] * 57.296;
//    
//        
//     }
  } else {
     rate[0] = (uint16_t)((gAlgorithm.scaledSensors[YRATE] * 57.296 + DBC_RATE_OFFSET) * 142.85);
     rate[1] = (uint16_t)((gAlgorithm.scaledSensors[XRATE] * 57.296 + DBC_RATE_OFFSET) * 142.85);
     rate[2] = (uint16_t)((gAlgorithm.scaledSensors[ZRATE] * 57.296 + DBC_RATE_OFFSET) * 142.85);
     
     buffer.b.x_value = ((rate[0] & 0xff) << 8) | ((rate[0] & 0xff00) >> 8);
     buffer.b.y_value = ((rate[1] & 0xff) << 8) | ((rate[1] & 0xff00) >> 8);
     buffer.b.z_value = ((rate[2] & 0xff) << 8) | ((rate[2] & 0xff00) >> 8);
     
     buffer.b.counter = dbc_rate_cnt % 16;
     buffer.b.reserved = 0;
     
     memcpy((void *)csum_buf, (void *)&(dbc_desc->tx_identifier.r), 2);
     memcpy((void *)&csum_buf[2], (void *)&(buffer.r), dbc_desc->tx_payload_len);
     
     buffer.b.csum = checksum(csum_buf, 9);
     
     memcpy((void *)dbc_desc->tx_buffer.Data, (void *)&(buffer.r), dbc_desc->tx_payload_len);
  }
    
  return 1;
}

static uint8_t dbc_send_accel(void)
{
  struct dbc_tx_desc *dbc_desc;
  DBC_SG_PAYLOAD buffer;
  uint16_t accel[3];
  uint8_t csum_buf[10];
  
  if (find_dbc_desc(&dbc_desc) == 0)
    return 0;
   
  dbc_desc->tx_payload_len = DBC_PAYLOAD_LEN;
  dbc_desc->tx_identifier.b.can_id = ACEINNA_DBC_MESSAGE_ACCEL_ID;
  
  dbc_desc->tx_pkt_ready = DESC_OCCUPIED;
  dbc_xl_cnt++;
  
  dbc_desc->tx_buffer.StdId = dbc_desc->tx_identifier.r;
  dbc_desc->tx_buffer.IDE = CAN_ID_STD;
  dbc_desc->tx_buffer.RTR = CAN_RTR_Data;
  dbc_desc->tx_buffer.DLC = dbc_desc->tx_payload_len;
  
  if( UcbGetSysType() > IMU_9DOF_SYS ) {
//      if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_SLOPE_SENSOR) {
//          slope_data.pitch = (uint32_t)((gKalmanFilter.eulerAngles[PITCH] * 57.296 + 250.00) * 32768);
//          slope_data.roll = (uint32_t)((gKalmanFilter.eulerAngles[ROLL] * 57.296 + 250.00) * 32768);
//          
//          slope_data.pitch_compensation = 0;
//          slope_data.pitch_merit = 0;
//          slope_data.roll_compensation = 0;
//          slope_data.roll_merit = 0;
//          slope_data.measure_latency = 0;
//  
//          memsic_j1939_send_slope_sensor(&slope_data);  
//      }
//  
//      if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_ACCELERATOR) {
//        accel_data.acceleration_x = (uint16_t)(((gKalmanFilter.correctedAccel_B[X_AXIS] * GRAVITY) + 320.00) * 100);
//        accel_data.acceleration_y = (uint16_t)(((gKalmanFilter.correctedAccel_B[Y_AXIS] * GRAVITY) + 320.00) * 100);
//        accel_data.acceleration_z = (uint16_t)(((gKalmanFilter.correctedAccel_B[Z_AXIS] * GRAVITY) + 320.00) * 100);
//     }
//  
//     if (gEcuConfigPtr->packet_type & MEMSIC_SAE_J1939_PACKET_ANGULAR_RATE) {
//        rate[0] = gKalmanFilter.correctedRate_B[Y_AXIS] * 57.296;
//        rate[1] = gKalmanFilter.correctedRate_B[X_AXIS] * 57.296;
//        rate[2] = gKalmanFilter.correctedRate_B[Z_AXIS] * 57.296;
//    
//        
//     }
  } else {
    accel[0] = (uint16_t)(((gAlgorithm.scaledSensors[XACCEL] * GRAVITY) + DBC_ACCEL_OFFSET) * 1000.00);
    accel[1] = (uint16_t)(((gAlgorithm.scaledSensors[YACCEL] * GRAVITY) + DBC_ACCEL_OFFSET) * 1000.00);
    accel[2] = (uint16_t)(((gAlgorithm.scaledSensors[ZACCEL] * GRAVITY) + DBC_ACCEL_OFFSET) * 1000.00); 
     
    buffer.b.x_value = ((accel[0] & 0xff) << 8) | ((accel[0] & 0xff00) >> 8);
    buffer.b.y_value = ((accel[1] & 0xff) << 8) | ((accel[1] & 0xff00) >> 8);
    buffer.b.z_value = ((accel[2] & 0xff) << 8) | ((accel[2] & 0xff00) >> 8);
    
    
    buffer.b.counter = dbc_xl_cnt % 16;
    buffer.b.reserved  = 0;
    memcpy((void *)csum_buf, (void *)&(dbc_desc->tx_identifier.r), 2);
    memcpy((void *)&csum_buf[2], (void *)&(buffer.r), dbc_desc->tx_payload_len);
     
    buffer.b.csum = checksum(csum_buf, 9);
        
    memcpy((void *)dbc_desc->tx_buffer.Data, (void *)&(buffer.r), dbc_desc->tx_payload_len);
  }
    
  return 1;
}

void dbc_process(void)
{
  struct dbc_tx_desc *tx_desc;
  uint8_t result;
  
  dbc_send_rate();
  dbc_send_accel();
  
  tx_desc = curr_tx_desc;
  while (tx_desc->tx_pkt_ready != DESC_IDLE) 
  {
    if (tx_desc->tx_pkt_ready == DESC_PENDING) {
      tx_desc = tx_desc->next;
      if (tx_desc == curr_tx_desc)
        return;
      else
        continue;
    }
      
    if ((result = send_dbc_packet(tx_desc)) != CAN_TxStatus_NoMailBox) {
      tx_desc->tx_pkt_ready = DESC_PENDING;
      tx_desc = tx_desc->next;
    }
    else
      return;
  }
  
  return;
}

void dbc_transmit_isr(void)
{
  struct dbc_tx_desc *desc;
  
  desc = curr_tx_desc;
  
  if ((desc == NULL) || (desc->next == NULL) || (desc->tx_pkt_ready == DESC_IDLE))
    return;
  
  desc->tx_pkt_ready = DESC_IDLE;
  
  if (desc->next->tx_pkt_ready == DESC_PENDING)
    curr_tx_desc = curr_tx_desc->next;
  
  return;
}
#endif 