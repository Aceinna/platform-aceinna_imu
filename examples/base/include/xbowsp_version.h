/*****************************************************************************
 * @file xbowsp_version.h
 *
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * $Rev: 17514 $
 * @date: 2011-03-02 13:36:09 -0800 (Wed, 02 Mar 2011) $
 * @author: by-tdelong $
 * @brief Version definition based on UCB serial protocol.
 ******************************************************************************/
#ifndef VERSION_H
#define VERSION_H


/** @fn xbowsp_version.h
 *	@brief	contains versioning data
 *
 * Version Data = Major, Minor, Patch, Stage and Build#
 * 	- MajorVersion changes may introduce serious incompatibilities.
 *  - MinorVersion changes may add or modify functionality,
 *    but maintain backward compatibility with previous minor versions.
 *  - Patch level changes reflect bug fixes and internal modifications with
 *    little effect on the user.
 *  - The build stage is one of the following:
 *		0=release candidate
 *		1=development
 *		2=alpha
 *		3=beta.
 *	- The buildNumber is incremented with each engineering firmware build.
 *
 * The buildNumber and stage for released firmware are both zero.
 * The final beta candidate is v.w.x.3.y, which is then changed to v.w.x.0.1 to create the
 * first release candidate.  The last release candidate is v.w.x.0.z, which is then
 * changed to v.w.x.0.0 for release.
 *
 *  @author David Ammerlaan
 *
 * 	@version 10.23.2006	DRA	initial creation.
 *	@n		 04.20.2007	DRA	ECO release
 *
 */

// DO NOT CHANGE THESE NUMBERS FROM ZERO!  CAUSES A CONFLICT WITH
//   IMUTest RESULTING IN ACCELEROMETER VALUES THAT ARE FLIPPED (WHAT
//   SHOULD BE POSITIVE BECOMES NEGATIVE).
#define VERSION_MAJOR 0
#define VERSION_MINOR 0
#define VERSION_PATCH 0
#define VERSION_STAGE 0
#define VERSION_BUILD 0

#define VERSION_MAJOR_NUM 18
#define VERSION_MINOR_NUM 1
#define VERSION_PATCH_NUM 0
#define VERSION_STAGE_NUM 9
#define VERSION_BUILD_NUM 0

/// Software-version/part-number
// changing the version number changes the CRC for the EEPROM memory so to avoid
// BIT failure the CRC must be updated if the version number is incremented
// Note: if the part number is changed before calibration the cal calculates
// and sets the CRC in EEPROM
//
//                                    1         2
//                           12345678901234567890
#define  SOFTWARE_PART      "5020-1381-18.1.09a "
#define  SOFTWARE_PART_LEN  19
#define  VERSION_STR        SOFTWARE_PART   /* using the version defined in the protocol version, xbowsp_version.h  */
#define  N_VERSION_STR      128

#endif


// 17.0.1 --
// 17.0.2 -- Added LS Magnetic-Alignment algorithm
// 17.0.3 -- Added algorithm changes that include the following:
//           a) Sensor settings (800Hz accel w/ 400 Hz LPF and Rate-Sensor with 50 Hz filter)
//           b) Modifications to the EKF to match the latest Matlab model (sims match)
// 17.0.4 -- Committing Feng's changes that reduce the ODR of the acceleromter and
//           incorporate the 50 Hz LPF that runs at 400 Hz
// 17.0.5 -- Changing the ODR of the Origin GPS to 5 Hz
// 17.0.6 -- Reset the Origin GPS to 1 Hz.  Changed update so only the sections of P that pertain
//           to attitude and rate-bias are updated during an Update_AHRS function call. Corrected
//           how GPS rollover is handled.
// 17.0.7 -- Only major functional difference from 17.0.4 is the change in how P is calculated.  The
//           rest of the changes are code cleanup.
// 17.0.8 -- Changes up to, but not including, the high/low gain switching based on system acceleration
// 17.1.0  --
// 17.1.0a -- release version (17.1.0) with higher user-communication baud-rate capability
// 17.1.0b -- release version (17.1.0a) with additional filtering (2, 5, 10, 20, 25, 40, 50, inf)
//            as well as the ability to provide corrected and uncorrected rate-sensor signals
//            in the A1 packet.  Additionally, added a more reliable check on baud-rate/ODR/
//            message-type combination (restricting a combination if the communication's duty-
//            cycle is greater than 80%).
// 17.1.0c -- Ability to change the orientation via SPI as well as read the orientation setting.
//            Remove the mod-functions (replaced with if-statements) in the calibration and filtering
//            routines (these still remain in the GPS, WMM, and fixed-point math routines).
// 17.2.0 -- Sync capabilities added for Topcon and John Deere
// 18.0.1 -- Contains the cascaded second-order filters
// 18.1.0b -- Rolled in the changes to fix the sampling-induced accelerometer bias/SF issue
// 18.1.1 -- Changes to make the SPI communication register read/write work properly (NOTE: the burst
//           message is configurable but this feature has been disabled until it is needed)
// 18.1.2 -- Changed the way the BMI rate-sensor temperature word is handles.  Converted from BMI
//           'counts' to Maxim 'counts' at the point that 'reading' is populated.  This allow all
//           previous code (related to temperature) to operate as originally written and the
//           original calibration equation (based on Maxim sensor) to apply as well.
// 18.1.3 -- Added fix to the SPI implementation to prevent the SPI peripheral from 'getting confused'.
//           Moved RS read to the original location (from its last location at the end of the accel
//           callback).  This somehow prevented the SPI from missing a clock.
// 18.1.4 -- Added fix to AHRS sampling in taskDataAcquisition (100 Hz if AHRS selected/200 Hz for IMU).
// 18.1.5 -- Fixed to VG/AHRS sampling and integration problems in taskDataAcquisition (100 Hz if AHRS
//           selected/200 Hz for IMU).
// 18.1.6 -- Modified the BMI160 sensor read to make it shorter.  Angular-rate data is read at 800 Hz with
//           the temperature readings read at a 4 Hz rate (the complete buffer is read every 200 cycles).
// 18.1.7 -- Added the capabilty to select the pre-filter (AAF), removed the high-gain indicator when the
//           unit is in low-gain mode and linAccelSwitch was activated, fixed the BIT error caused by an
//           improper read of the BMI part number.
//
