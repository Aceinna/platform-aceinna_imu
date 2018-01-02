/** ***************************************************************************
 * @file calc_airData.h DMU380 air data calculations
 * @Author denglish
 * @date   2011-02-09 20:20:16 -0800 (Wed, 09 Feb 2011)
 * @rev 17465
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef CALC_AIRDATA_H
#define CALC_AIRDATA_H

/// raw sensor order
enum {
     SPRESS, 
     DPRESS, 
     OAT,
     SPRESSTEMP,
     DPRESSTEMP, 
     OATTEMP,
     SPRESSCUR, 
     DPRESSCUR,
     SIZE_AIRDATA_SENSOR
};

enum {
     TAS,CAS,
     CALTI,
     PALTI,
     DALTI,
     ALTIRATE, 
     OATC,
     SLIPSKID,
     MACH, 
     AIRSPEEDRATE,
     UNFILTERED_VS,
     ALTIRATE_F,
     SIZE_AIRDATA_SOLUTION
};


/// Values of  airDataSensorFlag
#define NEW_STATIC_PRESSURE   	0
#define NEW_DYNAMIC_PRESSURE  	1
#define NEW_OAT               	2
#define COMPUTE_MISCELLANEOUS   3
#define DO_NOTHING              4

typedef struct {
	unsigned int airDataSensorFlag;
	unsigned int rawSensors[SIZE_AIRDATA_SENSOR];
	int          tempCompBias[3];     /// the current temp comp airData sensor bias
	double       scaledSensors[SIZE_AIRDATA_SENSOR];
	double       airDataSolution[SIZE_AIRDATA_SOLUTION]; /// TAS,CAS,CALTI,PALTI,DALTI,ALTIRATE, OATC, SLIPSKID,MACH  */
	double       timeTag;
	int          baroCorrection; /// the baro correction that is exactly used for BaroCorrected Alti in A3 packet*/
	unsigned int initVSFilter;
	unsigned int status;
} AlgorithmAirDataStruct;

/// bit position of status
#define NO_VALID_CAL_DATA  0x1
#define NO_VALID_OAT       0x2

#define P_BOARD_TEMP_BIAS   	289341440
#define SP_TEMP_BIAS   			4.740764012639749e+007
#define DP_TEMP_BIAS   			4.740764012639749e+007

#define P_BOARD_TEMP_SCALE   	9.203129712002413e-007
#define SP_TEMP_SCALE   		1.021544204075113e-008
#define DP_TEMP_SCALE   		1.021544204075113e-008

/*defines for computation*/
#define FEET2METER							 0.3048
#define KNOTS_2_METER_PER_SECOND             0.514444444
#define HPA2KPA							   	 0.1
#define SSOS							  (661.48 * KNOTS_2_METER_PER_SECOND)
#define STANDARD_PRESSURE_AT_SEA_LEVEL	 (1013.25 * HPA2KPA)
#define IN_HG_2_KPA                          3.386530748663101e+000
#define BARO_CORRECTION_SCALING_IN_EEPROM 1000
#define SOUND_SPEED_AT_SEA_LEVEL           340.3   ///< m/s
#define VALID_MIN_TAS                       20   ///< knots
#define VERTICAL_VEL_CAP                    250    ///< m/s
#define AIRSPEED_RATE_CAP                   90     ///< m/s/s
#define OAT_LOW_DEGREE_C_LIMIT             -55.0  ///< degree
#define OAT_HIGH_DEGREE_C_LIMIT             90.0   ///< degree 
#define OAT_BIT_ERROR_COUNT_LIMIT 		    50
#define BARO_CORRECTION_RANGE_LOW           22
#define BARO_CORRECTION_RANGE_HIGH          35

#define DYNAMIC_THRESHOLD     0.75
#define VS_OUTPUT_FG_H        0.0100
#define VS_OUTPUT_FG_L        0.0050


extern AlgorithmAirDataStruct algorithmAirData;
extern unsigned int rawAirDataSensors[SIZE_AIRDATA_SENSOR];

extern void genericFptFilter(double *currentInput, int *numTaps, double *inOld, double *outOld, double *NumerCoeff, double *DenomCoeff)  ;

extern void calculateAirDataSolution(void)      ;

extern void calibrateADCSensors(unsigned int *staticAndDynmicPressureIndex) ;
extern void filterSpSensorData(AlgorithmAirDataStruct *airData) ;
extern void calcPressureOrBaroAlti(AlgorithmAirDataStruct *airData, double *alti, double *baroCorr) ;
extern void calcVerticalSpeed(AlgorithmAirDataStruct *airData)    ;
extern void filterOATSensorData(AlgorithmAirDataStruct *airData) ;
extern void filterDpSensorData(AlgorithmAirDataStruct *airData) ;
extern void calcIAS(AlgorithmAirDataStruct *airData)         ;
extern void calcAirspeedRate(AlgorithmAirDataStruct *airData)    ;
extern void calcDensityAlti(AlgorithmAirDataStruct *airData)   ;
extern void calcMach(AlgorithmAirDataStruct *airData)       ;
extern void calcTAS(AlgorithmAirDataStruct *airData)  ;

extern double calcStandardTemperature(AlgorithmAirDataStruct *airData) ;
extern double calcSoundSpeed(AlgorithmAirDataStruct *airData)   ;
extern void filterADCVertSpd(AlgorithmAirDataStruct *airData) ;
extern void filterAirspeedRate(AlgorithmAirDataStruct *airData) ;
extern void calibrateOATSensors(void);


#endif /* CALC_AIRDATA */

