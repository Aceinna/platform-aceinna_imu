/** ***************************************************************************
 * @file BIT.c Sensor data built in test (BIT) manipulation functions
 *       and collection from internal and external sensors.
 * @Author Darren Liccardo
 * @date   Oct. 2005
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief Sensor data, ordering, and filtering is based on the
 *              productConfiguration.architecture field.
 *****************************************************************************/

#include <math.h>
#include <stdlib.h>
#include "BIT.h"
#include "BITStatus.h"
#include "timer.h"
#include "xbowsp_init.h"
#include "EKF_Algorithm.h"
#include "scaling.h" // MAXINT16_OVER_2PI 10430.3783505
#include "debug.h"

#include "qmath.h"

extern CalibrationStruct   gCalibration;
extern AlgorithmStruct     gAlgorithm;
extern ConfigurationStruct gConfiguration;

uint16_t recorded_count = 0;

void _handleComBIT(void);

/** ****************************************************************************
 * @name handleBITandStatus processes BIT and Status word logic.
 * @author Darren Liccardo -  Jan. 2006
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void handleBITandStatus()
{
    /// freeze com bits on failure to allow visibility if com was absent
    if( !gAlgorithm.bitStatus.BITStatus.bit.masterFail ) {
        _handleComBIT();
    }

    /// master bit logic
//	gAlgorithm.bitStatus.comBIT.bit.serialAError      = gAlgorithm.bitStatus.comSABIT.all  ? 1:0;
    gAlgorithm.bitStatus.comBIT.bit.serialBError      = gAlgorithm.bitStatus.comSBBIT.all  ? 1:0;
    gAlgorithm.bitStatus.BITStatus.bit.comError       = gAlgorithm.bitStatus.comBIT.all    ? 1:0;
    gAlgorithm.bitStatus.hwBIT.bit.environmentalError = gAlgorithm.bitStatus.hwEnvBIT.all  ? 1:0;
//	gAlgorithm.bitStatus.hwBIT.bit.powerError         = gAlgorithm.bitStatus.hwPwrBIT.all  ? 1:0;
    gAlgorithm.bitStatus.BITStatus.bit.hardwareError  = gAlgorithm.bitStatus.hwBIT.all     ? 1:0;
    gAlgorithm.bitStatus.swBIT.bit.algorithmError     = gAlgorithm.bitStatus.swAlgBIT.all  ? 1:0;
    gAlgorithm.bitStatus.swBIT.bit.dataError          = gAlgorithm.bitStatus.swDataBIT.all ? 1:0;
    gAlgorithm.bitStatus.BITStatus.bit.softwareError  = gAlgorithm.bitStatus.swBIT.all     ? 1:0;
    
    /// master status logic
    gAlgorithm.bitStatus.BITStatus.bit.comStatus      = (gAlgorithm.bitStatus.comStatus.all    & gConfiguration.comStatusEnable)      ? 1:0;
    gAlgorithm.bitStatus.BITStatus.bit.hardwareStatus = (gAlgorithm.bitStatus.hwStatus.all     & gConfiguration.hardwareStatusEnable) ? 1:0;
    gAlgorithm.bitStatus.BITStatus.bit.sensorStatus   = (gAlgorithm.bitStatus.sensorStatus.all & gConfiguration.sensorStatusEnable)   ? 1:0;
    gAlgorithm.bitStatus.BITStatus.bit.softwareStatus = (gAlgorithm.bitStatus.swStatus.all     & gConfiguration.softwareStatusEnable) ? 1:0;
    gAlgorithm.bitStatus.BITStatus.bit.masterStatus   = (gAlgorithm.bitStatus.BITStatus.all    & 0x1E00)                              ? 1:0;

#if 0
	/// external mag logic
	gAlgorithm.bitStatus.comStatus.bit.noExternalMagnetometer = gAlgorithm.ExtAidConnected ? 0:1;

	/// magnetic heading reference logic
	if (gAlgorithm.ExtAidConnected ||
       (gCalibration.productConfiguration.bit.hasMags &&
        gConfiguration.userBehavior.bit.useMags &&
        !gConfiguration.userBehavior.bit.freeIntegrate))
	   gAlgorithm.bitStatus.swStatus.bit.noMagnetometerheading = 0;
    else
	   gAlgorithm.bitStatus.swStatus.bit.noMagnetometerheading = 1;
#endif

	/// GPS track reference is done in driverGPSAllEntrance.cpp
}

/** ****************************************************************************
 * @name _handleComBIT LOCAL checks for SCI errors.  A high pass error counter
 *       errorCount tracks error hits and flags a hard fail bit. Errors flagged
 *       by the com drivers are sustained for one iteration here before being
 *       disabled.
 * @author Darren Liccardo -  Jan. 2006
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _handleComBIT()
{
	static uint16_t errorCount[2] = {0,0};
//	static union  SCI_BIT lastComSABIT lastComSBBIT;  ///< last com serial BITs
	int           i;

	/// if new bit is one and last bit was one then new bit set to zero,
	/// thus disabling the bit after one iteration
//	algorithm.bitStatus.comSABIT.all &= ~lastComSABIT.all;
//	algorithm.bitStatus.comSBBIT.all &= ~lastComSBBIT.all;

	if(recorded_count != gAlgorithm.counter)
	{	/// if there was an error, it has been reported
		/// if this or the last iteration contained an error then consider it
		/// contributing to the failure
//		if(gAlgorithm.bitStatus.comSABIT.all || lastComSABIT.all)
//            errorCount[0] += 2;
//		if(gAlgorithm.bitStatus.comSBBIT.all || lastComSBBIT.all)
//            errorCount[1] += 2;

//		lastComSABIT.all = algorithm.bitStatus.comSABIT.all;
//		lastComSBBIT.all = algorithm.bitStatus.comSBBIT.all;
	}
/*	else
	{ @brief
		we just recorded a new error. do nothing. when the error has been
		reported, the if statement above will pass: the error count will
		br updated and the flag(s) cleared in the following iteration.
	}
*/

	for(i = 0; i < 2; i++) {
      if( errorCount[i] > COM_BIT_ERROR_COUNT_LIMIT ) {// 1000
//			HARDWARE_BIT = 1;
			/// make sure errors are lit!
//			gAlgorithm.bitStatus.comSABIT.all |= lastComSABIT.all;
//			gAlgorithm.bitStatus.comSBBIT.all |= lastComSBBIT.all;
			gAlgorithm.bitStatus.BITStatus.bit.masterFail = 1;
			errorCount[i] = COM_BIT_ERROR_COUNT_LIMIT + 1; ///< freeze 1000
		} else if(errorCount[i] > 0) {
			errorCount[i]--;
		}
	}
}


/** ****************************************************************************
 * @name setMagAlignOutOfBoundsBIT API flags the mag align out of bounds BIT.
 * @author Darren Liccardo -  Jan. 2006
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void setMagAlignOutOfBoundsBIT(void)
{
	if(abs(gConfiguration.hardIronBias[0]) > HARD_IRON_LIMIT ||
	   abs(gConfiguration.hardIronBias[1]) > HARD_IRON_LIMIT ||
	   abs(gConfiguration.softIronScaleRatio - IRON_SCALE)>SOFT_IRON_LIMIT ||
	   gConfiguration.softIronScaleRatio == 0 )  /// work around because abs(-32768) returns zero!
		gAlgorithm.bitStatus.swDataBIT.bit.magAlignOutOfBounds  = 1;
	else
        gAlgorithm.bitStatus.swDataBIT.bit.magAlignOutOfBounds  = 0;
}

/** ****************************************************************************
 * @name setLevelingOutofBoundsBIT API flags the mag leveling out of bounds
 * @author Darren Liccardo -  Jan. 2006
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void setLevelingOutofBoundBIT(void)
{
   if(abs(gConfiguration.OffsetAnglesExtMag[0]) >
     (int)(ROLL_INCIDENCE_LIMIT * MAXINT16_OVER_2PI) ||
      abs(gConfiguration.OffsetAnglesExtMag[1]) >
     (int)(ROLL_INCIDENCE_LIMIT * MAXINT16_OVER_2PI) )
   {
      gAlgorithm.bitStatus.swDataBIT.bit.magAlignOutOfBounds = 1;
   }
   else
      gAlgorithm.bitStatus.swDataBIT.bit.magAlignOutOfBounds = 0;
}

#include "taskUserCommunication.h"

/** ****************************************************************************
 * @name handleOverRange API flags the sensor over-range status and bit. After
 *       quasi-static condition is met (rates<threshold) then an algorithm
 *       restart is performed if user enabled.
 * @author Darren Liccardo -  Jan. 2006
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void handleOverRange(void)
{
	static uint16_t overRangeCount = 0;
	int           i;

    uint8_t sysRange = getSensorRange();

    // Acceleration
    for(i = X_AXIS; i <= Z_AXIS; i++) {
        if( fabs(gAlgorithm.scaledSensors[XACCEL+i]) > SENSOR_RANGE_Q27[sysRange][0]) {
			overRangeCount += 2;
			gAlgorithm.bitStatus.sensorStatus.bit.overRange = 1;
		}

        // Place a hard limit on the sensor (450 and 225 deg/sec) -- Q27 values used because using
        //   the non-q27 values resulted in a limit that was lower than the specified limit.
        if( gAlgorithm.scaledSensors_q27[XACCEL+i] > SENSOR_LIMIT_q27[sysRange][0] ) {
            gAlgorithm.scaledSensors_q27[XACCEL+i] = SENSOR_LIMIT_q27[sysRange][0];
        } else if( gAlgorithm.scaledSensors_q27[XACCEL+i] < -SENSOR_LIMIT_q27[sysRange][0] ) {
            gAlgorithm.scaledSensors_q27[XACCEL+i] = -SENSOR_LIMIT_q27[sysRange][0];
        }
        gAlgorithm.scaledSensors[XACCEL+i] = q27ToDouble( gAlgorithm.scaledSensors_q27[XACCEL+i] );
	}

    // Angular rate
    for(i = X_AXIS; i <= Z_AXIS; i++) {
        if( fabs(gAlgorithm.scaledSensors[XRATE+i]) > SENSOR_RANGE_Q27[sysRange][1]) {
			overRangeCount += 2;
			gAlgorithm.bitStatus.sensorStatus.bit.overRange = 1;
		}

        // Place a hard limit on the sensor (450 and 225 deg/sec) -- Q27 values used because using
        //   the non-q27 values resulted in a limit that was lower than the specified limit.
        if( gAlgorithm.scaledSensors_q27[XRATE+i] > SENSOR_LIMIT_q27[sysRange][1] ) {
            gAlgorithm.scaledSensors_q27[XRATE+i] = SENSOR_LIMIT_q27[sysRange][1];
        } else if( gAlgorithm.scaledSensors_q27[XRATE+i] < -SENSOR_LIMIT_q27[sysRange][1] ) {
            gAlgorithm.scaledSensors_q27[XRATE+i] = -SENSOR_LIMIT_q27[sysRange][1];
        }
        gAlgorithm.scaledSensors[XRATE+i] = q27ToDouble( gAlgorithm.scaledSensors_q27[XRATE+i] );
	}

	/// over-range trigger - with algo restart flag
	if( overRangeCount > OVER_RANGE_COUNT_LIMIT &&
       gConfiguration.userBehavior.bit.restartOnOverRange ) {

		gAlgorithm.bitStatus.swAlgBIT.bit.overRange   = 1;
		gAlgorithm.bitStatus.BITStatus.bit.masterFail = 1;

		/// check for quasi-static condition
//		for(i = 0; i < 3; i++)
//           if( IQ27abs(gAlgorithm.filteredRates[i]) > IQ27(QUASI_STATIC_OVER_RANGE_RATE) )
                //return;
		//TODO: check mag accel is near 1
		gAlgorithm.bitStatus.swAlgBIT.bit.overRange   = 0;
		gAlgorithm.bitStatus.BITStatus.bit.masterFail = 0;
		overRangeCount = 0;

        // Set the flags to RESTART the algorithm
        InitializeAlgorithmStruct(&gAlgorithm);
	} else if(overRangeCount > 0)
        overRangeCount--;
	else
        gAlgorithm.bitStatus.sensorStatus.bit.overRange = 0;
}

 /** ****************************************************************************
 * @name CollectSensorData gets sensor data Based on product architecture, the
 * data is filtered and mapped appropriately.
 * @brief The first initializeCount calls initialize filtered values to the sensor
 * data collected.
 * @author Darren Liccardo -  Jan. 2006
 * @param N/A
 * @retval N/A
 ******************************************************************************/
//#pragma CODE_SECTION("ramfuncs");
void collectSensorData(void)
{
	uint16_t     i;
//    Uint16     intCounts[11];
//	Uint32     extCounts[16];
	static int filterShift[6] = {0}; ///< initializes the filters on the initializeCount calls
    /// the number of inerations used to initialize filters. (should be at least
    /// 2, see next comment)
	static int initializeCount = 2;

	/// bit data
//	algorithm.bitRawSensors[PCBTEMP] = extCounts[1] >> 7;

	/// after initializeCount calls, load filter shift values
	if( !filterShift[0] ) {
		initializeCount--;
		if( !initializeCount )
			for(i = 0; i < 6; i++)
				filterShift[i] = TEMP_FILTER[gCalibration.productConfiguration.bit.architecture][i];
	}
}

/** ****************************************************************************
 * @name getTopERROR()
 * @brief return a packet Master error and status word
 * TRACE:
 * [SDD_BIT_PKT_TOPBIT <-- SRC_BIT_GET_TOPBIT]
 * @param N/A
 * @retval  16 bit top word of hierarchical errors
 ******************************************************************************/
uint16_t getTopERROR(void)
{
	return gAlgorithm.bitStatus.BITStatus.all;
}
/** ****************************************************************************
 * @name lowPass API fixed point single-pole IIR filter with resolution
 *			  adjustment and shifted coefficients (powers of two only).
 * @author Darren Liccardo -  Jan. 2006
 * @param {out] out - points to the filter output
 * @param [in] input - the current sample
 * @param [in] resolution - the number of addition bits of resolution
 * @param [in] 1/2^shiftCoef is the coefficient on the input
 * @retval N/A
 ******************************************************************************/
void lowPass(uint32_t *out,
             uint32_t input,
             int    resolution,
             int    shiftCoef)
{
	*out += (uint32_t)((input << resolution) - *out) >> shiftCoef;
}

/** ****************************************************************************
 * @name  lowPass2Pole API fixed point double-pole IIR filter with resolution
 *		  adjustment and shifted coefficients (powers of two only).
 * @author Darren Liccardo -  Jan. 2006
 * @param {out] out - points to the filter output
 * @param [in] lastInput - the previous sample
 * @param [in] input - the current sample
 * @param [in] resolution - the number of addition bits of resolution
 * @param [in] 1/2^shiftCoef is the coefficient on the input
 * @param [in] 1/2^shiftCoefLast  is the coefficiant on the last sample.
 * @retval N/A
 ******************************************************************************/
void lowPass2Pole(int32_t *out,
                  int32_t *lastInput,
                  int32_t input,
                  int   resolution,
                  int   shiftCoef,
                  int   shiftCoefLast)
{
	int32_t shiftedInput = input << resolution;
	*out += ((shiftedInput - *out) >> shiftCoef) + ((*lastInput - *out) >> shiftCoefLast);
	*lastInput = shiftedInput;
}
