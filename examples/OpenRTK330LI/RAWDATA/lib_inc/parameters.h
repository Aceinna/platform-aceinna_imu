/** ***************************************************************************
  * @file xbowsp_fields.h functions for handling DMU packets
  * @Author denglish
  * @date  2011-02-09 20:58:18 -0800 (Wed, 09 Feb 2011)
  * @rev 17469
  * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
  *
  * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY O ANY
  * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  * PARTICULAR PURPOSE.
  *
  *****************************************************************************/
#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "configuration.h"
#include "ucb_packet.h"
#include <stdint.h>
 
extern BOOL   	CheckPortUsage 			  (ConfigurationStruct *proposedConfiguration) ;
extern BOOL   	CheckPortBaudRate 		  (uint16_t portBaudRate) ;
extern BOOL   	CheckPacketRateDivider	  (uint16_t packetRateDivider) ;
extern BOOL		  CheckContPacketRate       (UcbPacketType outputPacket, uint16_t baudRate, uint16_t packetRateDivider) ;
extern uint8_t	CheckRamFieldData 		  (uint8_t numFields, uint16_t fieldId [], uint16_t fieldData [], uint16_t validFields []) ;
extern uint8_t	CheckEepromFieldData 	  (uint8_t numFields, uint16_t fieldId [], uint16_t fieldData [], uint16_t validFields []) ;
extern void		  SetFieldData			  (void) ;

#define CONFIGURATION_START                 0x0000
#define CRC_ID                              0x0000
#define LOWER_CONFIG_ADDR_BOUND				      0x0001  ///< lower configuration address boundary
#define PACKET_RATE_DIVIDER_FIELD_ID  		  0x0001	///< continuous packet rate divider
#define BAUD_RATE_USER_ID             	  	0x0002	///< continuous packet rate divider
#define PACKET_TYPE_FIELD_ID  				      0x0003	///< continuous packet type
#define ORIENTATION_FIELD_ID  				      0x0007	///< user defined axis orientation
#define USER_BEHAVIOR_FIELD_ID				      0x0008	///< user behaviour switches
#define TURN_SWITCH_THRESHOLD_FIELD_ID		  0x000D	///< turn switch threshold
#define HARDWARE_STATUS_ENABLE_FIELD_ID 	  0x0010	///< hardware status enable
#define COM_STATUS_ENABLE_FIELD_ID  		    0x0011	///< communication status enable
#define SOFTWARE_STATUS_ENABLE_FIELD_ID	 	  0x0012	///< software status enable
#define SENSOR_STATUS_ENABLE_FIELD_ID  		  0x0013	///< sensor status enable
#define BARO_CORRECTION_FIELD_ID  			    0x0016	///< barometer correction
#define OFFSET_ANGLES_EXT_MAG_FIELD_ID		  0x0017	///< external mag roll offset
#define OFFSET_ANGLES_ALIGN_FIELD_ID  		  0x0018	///< roll offset alignment
#define OFFSET_ROLL_ALIGN_FIELD_ID          0x0019	///< roll offset alignment
#define OFFSET_PITCH_ALIGN_FIELD_ID         0x001A	///< pitchoffset alignment
#define OFFSET_YAW_ALIGN_FIELD_ID           0x001B	///< yaw offset alignment
#define HARD_IRON_BIAS_EXT_FIELD_ID  		    0x001C	///< external mag X hard iron bias
#define SOFT_IRON_SCALE_RATIO_EXT_FIELD_ID  0x001E	///< external mag soft iron scale ratio
#define SOFT_IRON_ANGLE_EXT_FIELD_ID  		  0x001F	///< external mag soft iron angle
#define PORT_1_USAGE_FIELD_ID  		    	    0x0021
#define PORT_2_USAGE_FIELD_ID               0x0022
#define PORT_3_USAGE_FIELD_ID               0x0023
#define PORT_4_USAGE_FIELD_ID               0x0024
#define PORT_1_BAUD_RATE_FIELD_ID           0x0025
#define PORT_2_BAUD_RATE_FIELD_ID           0x0026
#define PORT_3_BAUD_RATE_FIELD_ID           0x0027
#define PORT_4_BAUD_RATE_FIELD_ID           0x0028
#define REMOTE_MAG_SER_NO_FIELD_ID  		    0x0029
#define REMOTE_MAG_SER_NO_FIELD_ID_1  		  0x002A
#define ECU_ADDRESS_FIELD_ID                0x0032
#define ECU_BAUD_RATE_FIELD_ID              0x0033
#define UPPER_CONFIG_ADDR_BOUND				      0x0054	///< upper configuration address boundary

#define PRODUCT_CONFIGURATION_FIELD_ID		0x071C	///< outside of configuration, but needs to be read as a field

#define NUM_CONFIG_FIELDS					(UPPER_CONFIG_ADDR_BOUND - LOWER_CONFIG_ADDR_BOUND - 2)
#define BYTES_PER_CONFIG_FIELD				2

extern BOOL CheckOrientation (uint16_t orientation) ;
extern void DefaultPortConfiguration (void);
extern BOOL CheckBaroCorrection(int32_t baroCorrection) ;
extern BOOL WriteFieldData (void);

extern uint16_t appendAttitudeTrue (uint8_t *response, uint16_t index);
extern uint16_t appendCorrectedRates (uint8_t *response, uint16_t index) ;
extern uint16_t appendAccels (uint8_t *response, uint16_t index);
extern uint16_t appendChipAccels (uint8_t *response, uint16_t index, int chipId);
extern uint16_t appendCorrectedAccels (uint8_t *response, uint16_t index);
extern uint16_t appendTangentRates (uint8_t *response, uint16_t index);
extern uint16_t appendTangentAccels (uint8_t *response, uint16_t index);
extern uint16_t appendRates (uint8_t *response, uint16_t index);
extern uint16_t appendChipRates (uint8_t *response, uint16_t index, int chipId);
extern uint16_t appendMagReadings (uint8_t *response, uint16_t index);
extern uint16_t appendTemps (uint8_t *response, uint16_t index);
extern uint16_t appendChipTemps (uint8_t *response, uint16_t index, int chipId);
extern uint16_t appendInertialCounts (uint8_t *response, uint16_t index);
extern uint16_t appendMagnetometerCounts (uint8_t *response, uint16_t index);
extern uint16_t appendAllTempCounts (uint8_t *response, uint16_t index);
extern uint16_t appendRateTemp (uint8_t  *response, uint16_t index);
extern uint16_t appendGpsVel (uint8_t  *response, uint16_t index);
extern uint16_t appendGpsPos (uint8_t  *response, uint16_t index);
uint16_t        appendKalmanVel (uint8_t  *response, uint16_t index);
uint16_t        appendKalmanPos (uint8_t  *response, uint16_t index);
uint16_t        appendTemp (uint8_t  *response, uint16_t index);


// API fcns to load words and shorts into the byte buffers
extern uint32_t uint32ToBuffer(uint8_t *buffer, uint16_t index, uint32_t inWord);
extern uint32_t uint16ToBuffer(uint8_t *buffer, uint16_t index, uint16_t inWord);
#endif