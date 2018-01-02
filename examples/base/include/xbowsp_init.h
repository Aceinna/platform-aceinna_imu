/** ****************************************************************************
 * @file  xbowsp_init.h
 *
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * $Rev: 17479 $
 * @date: 2011-02-09 22:02:39 -0800 (Wed, 09 Feb 2011) $
 * @author: by-dan $
 * @brief -header file of xbowsp_init.c
 ******************************************************************************/
#ifndef XBOWSP_INIT_H
#define XBOWSP_INIT_H

#include "xbowsp_generaldrivers.h"
#include "xbowsp_algorithm.h"
//#include "sae_j1939.h"

#define ROLL_INCIDENCE_LIMIT  0x1000
#define PITCH_INCIDENCE_LIMIT 0x1000
#define HARD_IRON_LIMIT       8192 // 0.25 G
#define SOFT_IRON_LIMIT       6554 // 20%

extern void initConfigureUnit(void);
extern BOOL ValidPortConfiguration(ConfigurationStruct *proposedConfiguration);
extern void initAlgStruct(void);

extern AlgorithmStruct       gAlgorithm;
extern softwareVersionStruct bootFMversion;
extern ConfigurationStruct   gConfiguration;
extern CalibrationStruct     gCalibration;
extern softwareVersionStruct dupFMversion;
extern softwareVersionStruct ioupFMversion;
//extern EcuConfigurationStruct  gEcuConfig;

extern BOOL IsInternalGPS(void);

extern int16_t ConfigurationBaudRateGps(void);
extern void    SetConfigurationBaudRateGps(int16_t b);
extern int16_t ConfigurationProtocolGPS(void);
extern void    SetConfigurationProtocolGPS(int16_t p);
extern void    SetAlgorithmUseDgps(BOOL d);

uint8_t getSensorRange();
void SetIntVectorOffset(u32 offset);
#endif
