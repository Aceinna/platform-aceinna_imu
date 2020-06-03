/** ***************************************************************************
 * @file   UserConfiguration.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 ******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "algorithmAPI.h"
#include "magAPI.h"
#include "platformAPI.h"
#include "sensorsAPI.h"
#include "userAPI.h"
#include "UserMessagingCAN.h"
#include "UserConfiguration.h"
#include "canAPI.h"
#include "MagAlign.h"
#include "algorithm.h"
#include "BITStatus.h"
#include "sensorsAPI.h"
#include "sae_j1939.h"

// for EKFOutputDataStruct
#include "EKF_Algorithm.h"
EKF_OutputDataStruct *algo_res;
extern int initAlgo; 
uint32_t forced_packets = 0;

/** ***************************************************************************
 * @name  process_request() an API of processing request message
 * @brief decode identifier of incoming request message
 *
 * @param [in] desc, rx descriptor
 * @retval N/A
 ******************************************************************************/
void ProcessRequest(void *dsc)
{
  struct sae_j1939_rx_desc   *desc = (struct sae_j1939_rx_desc*)dsc;
  SAE_J1939_IDENTIFIER_FIELD *ident;
  uint8_t pf_val, req_pf_val, req_ps_val;
  uint8_t *command;
  
  // check desc
  if (desc == NULL)
    return;
  
  
  // check identifier
  ident = &(desc->rx_identifier);
  if (ident == NULL)
    return;
  
  // check receiving buffer
  if (desc->rx_buffer.RTR || !desc->rx_buffer.IDE || (desc->rx_buffer.DLC != 3)) 
      return;
  
  // check commands
  command = desc->rx_buffer.Data;
  if (command == NULL)
    return;
  
  pf_val = ident->pdu_format;
  req_pf_val = command[1];
  req_ps_val = command[2];
  
  if (pf_val != SAE_J1939_PDU_FORMAT_REQUEST)
    return;
  
  // dispatch the requests to the corresponding handlers
  switch(req_pf_val) {
    // software version request
    case SAE_J1939_PDU_FORMAT_254:
        {
         	if(req_ps_val == SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION){
            aceinna_j1939_send_software_version();
        }
        }
        break;
    // ecu id request
    case SAE_J1939_PDU_FORMAT_ECU:
        {
            aceinna_j1939_send_ecu_id();
        }
        break;
    // data packet request
    case SAE_J1939_PDU_FORMAT_DATA:
        forced_packets = 0;
        if(req_ps_val == SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR)
        {
            forced_packets |= ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR;
        }
        if(req_ps_val == SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE)
        {
            forced_packets |= ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE;
        }
        if(req_ps_val == SAE_J1939_GROUP_EXTENSION_ACCELERATION)
        {
            forced_packets |= ACEINNA_SAE_J1939_PACKET_ACCELERATION;
        }
        if(req_ps_val == SAE_J1939_GROUP_EXTENSION_MAG)
        {
            forced_packets |= ACEINNA_SAE_J1939_PACKET_MAG;
        }
        break;
    
    case SAE_J1939_PDU_FORMAT_GLOBAL:
      // status request
        if ((req_ps_val == gEcuConfigPtr->status_ps)) {
        STATUS_TEST_PAYLOAD status_bits;
        status_bits.masterStatus = (gBitStatus.BITStatus.all & 0x1E00) ? 1:0;
        status_bits.hardwareStatus = gBitStatus.hwStatus.all? 1 : 0;
        status_bits.softwareStatus = gBitStatus.swStatus.all? 1 : 0;
        status_bits.sensorStatus = gBitStatus.sensorStatus.all? 1 : 0;
        status_bits.unlocked1PPS = gBitStatus.hwStatus.bit.unlocked1PPS;
        status_bits.unlockedInternalGPS = gBitStatus.hwStatus.bit.unlockedInternalGPS;
        status_bits.noDGPS = gBitStatus.hwStatus.bit.unlockedInternalGPS;
        status_bits.unlockedEEPROM = gBitStatus.hwStatus.bit.unlockedEEPROM;
        status_bits.algorithmInit = gBitStatus.swStatus.bit.algorithmInit;
        status_bits.highGain = gBitStatus.swStatus.bit.highGain;
        status_bits.attitudeOnlyAlgorithm = gBitStatus.swStatus.bit.attitudeOnlyAlgorithm;
        status_bits.turnSwitch = gBitStatus.swStatus.bit.turnSwitch;
        status_bits.sensorOverRange = gBitStatus.sensorStatus.bit.overRange;
            aceinna_j1939_send_status_packet(ACEINNA_SAE_J1939_BUILTIN_STATUS, (void *)&status_bits);
      }
      // pasket rate request 
      else if ((req_ps_val == gEcuConfigPtr->packet_rate_ps)) {
        aceinna_j1939_send_packet_rate(gEcuConfigPtr->packet_rate);
      }
      // pasket type request 
      else if ((req_ps_val == gEcuConfigPtr->packet_type_ps)) {
        aceinna_j1939_send_packet_type(gEcuConfigPtr->packet_type);
      }
      // filter settings request 
      else if ((req_ps_val == gEcuConfigPtr->digital_filter_ps)) {
        aceinna_j1939_send_digital_filter(gEcuConfigPtr->accel_cut_off, gEcuConfigPtr->rate_cut_off);
      }
      // orientation settings request 
      else if ((req_ps_val == gEcuConfigPtr->orientation_ps)) {
        uint8_t bytes[2];
        bytes[0] = (gEcuConfigPtr->orien_bits >> 8) & 0xff;
        bytes[1] = (gEcuConfigPtr->orien_bits) & 0xff;
        aceinna_j1939_send_orientation(bytes);
      }

      break;
      // address claim request
    case SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM:
       process_request_pg(desc);
      break;
    default:
      break;
  }
  
  return;
}

/** ***************************************************************************
 * @name  send_tilt_sensor_data() an API of data packet generation
 * @brief build up MTLT's data packets and send out
 *
 * @param [in]
 * @retval N/A
 ******************************************************************************/
static int  outputCounter = 0;

void EnqeuePeriodicDataPackets(int latency, int sendPeriodicPackets)
{
  AUGULAR_RATE   angle_data;
  ACCELERATION_SENSOR accel_data;
  MAGNETIC_SENSOR     mag_data;
  int packets_to_send = 0;
  float rate[3];
  int i;
  
  outputCounter++;
  
  if (sendPeriodicPackets && gEcuConfig.packet_rate != 0){
      packets_to_send = gEcuConfigPtr->packet_type;
  }

  packets_to_send |= forced_packets;


   // acceleration packets
   if (packets_to_send & ACEINNA_SAE_J1939_PACKET_ACCELERATION) {
        double accelData[3];
        GetAccelData_mPerSecSq(accelData);
        accel_data.acceleration_x = (uint16_t)(((accelData[0]) + 320.00) * 100);
        accel_data.acceleration_y = (uint16_t)(((accelData[1]) + 320.00) * 100);
        accel_data.acceleration_z = (uint16_t)(((accelData[2]) + 320.00) * 100);

        accel_data.lateral_merit        = 3;
        accel_data.longitudinal_merit   = 3;
        accel_data.vertical_merit       = 3;
        accel_data.transmit_rate        = 2;
        accel_data.rsvd                 = 0;

        aceinna_j1939_send_acceleration(&accel_data);
   }
  
   if (packets_to_send & ACEINNA_SAE_J1939_PACKET_MAG) {
        double magData[3];
        GetMagData_G(magData);
        mag_data.mag_x = (uint16_t)(((magData[0]) + 8.00)/0.00025);
        mag_data.mag_y = (uint16_t)(((magData[1]) + 8.00)/0.00025);
        mag_data.mag_z = (uint16_t)(((magData[2]) + 8.00)/0.00025);
        mag_data.unuzed = 0;
        aceinna_j1939_send_mags(&mag_data);
   }
  
   if (packets_to_send & ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE) {
        double rateData[3];
        GetRateData_degPerSec(rateData);
        rate[0] = (float)(rateData[0]);
        rate[1] = (float)(rateData[1]);
        rate[2] = (float)(rateData[2]);

     for (i = 0; i < 3; i++)
     {
       if (rate[i] < -250.00)
         rate[i] = -250.00;

       if (rate[i] > 250.00)
         rate[i] = 250.00;
     }
     angle_data.pitch_rate = (uint16_t)((rate[0] + 250.00) * 128);
     angle_data.roll_rate  = (uint16_t)((rate[1] + 250.00) * 128);
     angle_data.yaw_rate   = (uint16_t)((rate[2] + 250.00) * 128);
     angle_data.pitch_merit = 3;
     angle_data.roll_merit  = 3;
     angle_data.yaw_merit   = 3;
     angle_data.rsvd        = 0;
     angle_data.measurement_latency = latency/5;

     aceinna_j1939_send_angular_rate(&angle_data);
   }

  forced_packets = 0;

   return;  
}

/** ***************************************************************************
 * @name  ProcessEcuCommands 
* @brief decode incoming SET packets and put the request into the corresponding 
 *       handlers
 * @param [in] command, SET request
 *             ps, pdu specific value
 *             addr, host address
 * @retval 1 successfuk or 0 failure
 ******************************************************************************/
void ProcessEcuCommands(void * command, uint8_t ps, uint8_t addr)
{ 
  // SET request is NULL
  if (command == NULL)
    return;
  
  if ((ps == SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION) ||  
          (ps == gEcuConfigPtr->save_cfg_ps)) 
  {
      // Save configuration command
      COMMAND_SET_PAYLOAD * pld = (COMMAND_SET_PAYLOAD *)command;
      if (pld->request == ACEINNA_SAE_J1939_REQUEST && pld->dest_address == *(uint8_t *)gEcu->addr){ 
          ECU_ADDRESS_ENTRY target;
          SaveEcuConfig(gEcuConfigPtr);
          target.address  = addr;
          target.category = _ECU_MASTER;
          aceinna_j1939_send_algrst_cfgsave(&target, 0, 1);  
      }
  }
  else if  (ps == gEcuConfigPtr->packet_rate_ps)
  {
    // Update packet rate command
    RATE_CONFIG_PAYLOAD * pld = (RATE_CONFIG_PAYLOAD *)command;
    if (pld->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->packet_rate       =  pld->odr;
      gUserConfiguration.ecuPacketRate =  pld->odr;
    }
  }
  else if  (ps == gEcuConfigPtr->packet_type_ps)
  {
    // Update set of transmitted packets
    PACKET_TYPE_PAYLOAD * pld = (PACKET_TYPE_PAYLOAD *)command;
    if(pld->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->packet_type       = pld->type_bits.r;
      gUserConfiguration.ecuPacketType = pld->type_bits.r;
    }
  }
  else if  (ps == gEcuConfigPtr->digital_filter_ps)
  {
    // Update cutoff frequencies for digital filters
    DIGITAL_FILTER_PAYLOAD * pld = (DIGITAL_FILTER_PAYLOAD *)command;
    if (pld->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->rate_cut_off            = pld->rate_cutoff;
      gEcuConfigPtr->accel_cut_off           = pld->accel_cutoff;
      gUserConfiguration.ecuFilterFreqAccel  = pld->accel_cutoff;
      gUserConfiguration.ecuFilterFreqRate   = pld->rate_cutoff;
      platformSelectLPFilter(RATE_SENSOR,  gEcuConfigPtr->rate_cut_off, TRUE);
      platformSelectLPFilter(ACCEL_SENSOR, gEcuConfigPtr->accel_cut_off, TRUE);
    }
  }
  else if  (ps == gEcuConfigPtr->orientation_ps)
  {
    // Update unit orientation
    ORIENTATION_SETTING * pld = (ORIENTATION_SETTING *)command;
    if (pld->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->orien_bits = pld->orien_bits[0] << 8 | pld->orien_bits[1];
      if(platformApplyOrientation(gEcuConfigPtr->orien_bits)){
          gUserConfiguration.ecuOrientation = gEcuConfigPtr->orien_bits;
      }
    }
  }   
  else if (ps == SAE_J1939_GROUP_EXTENSION_BANK0)
  {
      BANK0_PS_PAYLOAD * pld = (BANK0_PS_PAYLOAD *)command;
    if (pld->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->alg_reset_ps    = pld->alg_reset_ps;
      gEcuConfigPtr->status_ps       = pld->status_ps;
        gEcuConfigPtr->mag_align_ps    = pld->mag_align_ps;
      UpdateEcuConfig(gEcuConfigPtr, FALSE);
  }
  }
  else if (ps == SAE_J1939_GROUP_EXTENSION_BANK1)
  {
      // Reassign pdu-specific codes
      BANK1_PS_PAYLOAD * pld = (BANK1_PS_PAYLOAD *)command;
        if (pld->dest_address == *(uint8_t *)gEcu->addr) {
      gEcuConfigPtr->packet_rate_ps        = pld->packet_rate_ps;
      gEcuConfigPtr->packet_type_ps        = pld->packet_type_ps;
      gEcuConfigPtr->digital_filter_ps     = pld->digital_filter_ps;
      gEcuConfigPtr->orientation_ps        = pld->orientation_ps;
      gEcuConfigPtr->user_behavior_ps      = pld->user_behavior_ps;
      UpdateEcuConfig(gEcuConfigPtr, FALSE);
  }    
    }
  
}



/** ***************************************************************************
 * @name  ProcessDataPAcketss 
* @brief  decode and process incoming Vehicle data packets 
******************************************************************************/
void ProcessDataPackets(void *dsc)
{

}

// configure CAN controller for selective reception of CAN messages
/** ***************************************************************************
 * @name  ConfigureCANMessageFilters 
* @brief configure CAN controller for selective reception of CAN messages  
******************************************************************************/
void ConfigureCANMessageFilters()
{
     // initialize filter for ECU ID
    ConfigureCANMessageFilter(SAE_J1939_ECU_ID_BASE, SAE_J1939_ECU_FILTER_BASE_MASK);
    // initialize filter for control messages
    ConfigureCANMessageFilter(SAE_J1939_CONTROL1_ID_BASE, SAE_J1939_CONTROL1_FILTER_BASE_MASK);
    // initialize filter for requests
    ConfigureCANMessageFilter(SAE_J1939_REQUEST_ID_BASE, SAE_J1939_REQUEST_FILTER_BASE_MASK);
    // initialize filter for address claim
    ConfigureCANMessageFilter(SAE_J1939_ADDRESS_CLAIM_ID_BASE, SAE_J1939_ADDRESS_CLAIM_FILTER_BASE_MASK);
    // initialize filter for Bank 1 and Bank 0 commands
    ConfigureCANMessageFilter(ACEINNA_BANK_ID_BASE, ACEINNA_BANK_FILTER_BASE_MASK);
    // initialize Filter for incoming data messages
    ConfigureCANMessageFilter(VEHICLE_DATA_ID_BASE, VEHICLE_DATA_FILTER_BASE_MASK);
}
