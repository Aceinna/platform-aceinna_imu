/*
 * File:   MagAlign.h
 * Author: joemotyka
 *
 * Created on May 16, 2016, 8:22 PM
 */

#ifndef MAGALIGN_H
#define MAGALIGN_H

#include "GlobalConstants.h"

/* new mag-align progress status   */
#define MAG_ALIGN_STATUS_LEVEL_START  			0x6
#define MAG_ALIGN_STATUS_LEVEL_END    			0xF
#define MAG_ALIGN_STATUS_SAVE2EEPROM  			0xE
#define MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND   0x9
#define MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND      0xC
#define MAG_ALIGN_STATUS_TERMINATION                 0xB
#define MAG_ALIGN_STATUS_IDLE                        0x0

typedef struct {
    real hardIronBias[2];
    real softIronAngle;
    real softIronScaleRatio;
    real SF[4];
} MagAlignStruct;

extern MagAlignStruct gMagAlign;

#endif /* MAGALIGN_H */
