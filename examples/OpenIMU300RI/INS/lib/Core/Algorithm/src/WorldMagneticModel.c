/******************************************************************************  
* @file WorldMagneticModel.c
*
*******************************************************************************/
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

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include "osapi.h"
#include "WorldMagneticModel.h"
#include "WMMInternal.h"
#include "GpsData.h"

#include "GlobalConstants.h"


WMMtype_Ellipsoid     Ellip;
WMMtype_MagneticModel MagneticModel;

// WMM Coefficients valid from 2015-2020
float coeffs[91][6] = { {  0,  0,       0.0f,       0.0f,        0.0f,        0.0f },      // 1
                        {  1,  0,  -29438.5f,       0.0f,       10.7f,        0.0f },      // 2
                        {  1,  1,   -1501.1f,    4796.2f,       17.9f,      -26.8f },      // 3
                        {  2,  0,   -2445.3f,       0.0f,       -8.6f,        0.0f },      // 4
                        {  2,  1,    3012.5f,   -2845.6f,       -3.3f,      -27.1f },      // 5
                        {  2,  2,    1676.6f,    -642.0f,        2.4f,      -13.3f },      // 6
                        {  3,  0,    1351.1f,       0.0f,        3.1f,        0.0f },      // 7
                        {  3,  1,   -2352.3f,    -115.3f,       -6.2f,        8.4f },      // 8
                        {  3,  2,    1225.6f,     245.0f,       -0.4f,       -0.4f },      // 9
                        {  3,  3,     581.9f,    -538.3f,      -10.4f,        2.3f },      // 10
                        {  4,  0,     907.2f,       0.0f,       -0.4f,        0.0f },      // 1
                        {  4,  1,     813.7f,     283.4f,        0.8f,       -0.6f },      // 2
                        {  4,  2,     120.3f,    -188.6f,       -9.2f,        5.3f },      // 3
                        {  4,  3,    -335.0f,     180.9f,        4.0f,        3.0f },      // 4
                        {  4,  4,      70.3f,    -329.5f,       -4.2f,       -5.3f },      // 5
                        {  5,  0,    -232.6f,       0.0f,       -0.2f,        0.0f },      // 6
                        {  5,  1,     360.1f,      47.4f,        0.1f,        0.4f },      // 7
                        {  5,  2,     192.4f,     196.9f,       -1.4f,        1.6f },      // 8
                        {  5,  3,    -141.0f,    -119.4f,        0.0f,       -1.1f },      // 9
                        {  5,  4,    -157.4f,      16.1f,        1.3f,        3.3f },      // 20
                        {  5,  5,       4.3f,     100.1f,        3.8f,        0.1f },      // 1
                        {  6,  0,      69.5f,       0.0f,       -0.5f,        0.0f },      // 2
                        {  6,  1,      67.4f,     -20.7f,       -0.2f,        0.0f },      // 3
                        {  6,  2,      72.8f,      33.2f,       -0.6f,       -2.2f },      // 4
                        {  6,  3,    -129.8f,      58.8f,        2.4f,       -0.7f },      // 5
                        {  6,  4,     -29.0f,     -66.5f,       -1.1f,        0.1f },      // 6
                        {  6,  5,      13.2f,       7.3f,        0.3f,        1.0f },      // 7
                        {  6,  6,     -70.9f,      62.5f,        1.5f,        1.3f },      // 8
                        {  7,  0,      81.6f,       0.0f,        0.2f,        0.0f },      // 9
                        {  7,  1,     -76.1f,     -54.1f,       -0.2f,        0.7f },      // 30
                        {  7,  2,      -6.8f,     -19.4f,       -0.4f,        0.5f },      // 1
                        {  7,  3,      51.9f,       5.6f,        1.3f,       -0.2f },      // 2
                        {  7,  4,      15.0f,      24.4f,        0.2f,       -0.1f },      // 3
                        {  7,  5,       9.3f,       3.3f,       -0.4f,       -0.7f },      // 4
                        {  7,  6,      -2.8f,     -27.5f,       -0.9f,        0.1f },      // 5
                        {  7,  7,       6.7f,      -2.3f,        0.3f,        0.1f },      // 6
                        {  8,  0,      24.0f,       0.0f,        0.0f,        0.0f },      // 7
                        {  8,  1,       8.6f,      10.2f,        0.1f,       -0.3f },      // 8
                        {  8,  2,     -16.9f,     -18.1f,       -0.5f,        0.3f },      // 9
                        {  8,  3,      -3.2f,      13.2f,        0.5f,        0.3f },      // 40
                        {  8,  4,     -20.6f,     -14.6f,       -0.2f,        0.6f },      // 1
                        {  8,  5,      13.3f,      16.2f,        0.4f,       -0.1f },      // 2
                        {  8,  6,      11.7f,       5.7f,        0.2f,       -0.2f },      // 3
                        {  8,  7,     -16.0f,      -9.1f,       -0.4f,        0.3f },      // 4
                        {  8,  8,      -2.0f,       2.2f,        0.3f,        0.0f },      // 5
                        {  9,  0,       5.4f,       0.0f,        0.0f,        0.0f },      // 6
                        {  9,  1,       8.8f,     -21.6f,       -0.1f,       -0.2f },      // 7
                        {  9,  2,       3.1f,      10.8f,       -0.1f,       -0.1f },      // 8
                        {  9,  3,      -3.1f,      11.7f,        0.4f,       -0.2f },      // 9
                        {  9,  4,       0.6f,      -6.8f,       -0.5f,        0.1f },      // 50
                        {  9,  5,     -13.3f,      -6.9f,       -0.2f,        0.1f },      // 1
                        {  9,  6,      -0.1f,       7.8f,        0.1f,        0.0f },      // 2
                        {  9,  7,       8.7f,       1.0f,        0.0f,       -0.2f },      // 3
                        {  9,  8,      -9.1f,      -3.9f,       -0.2f,        0.4f },      // 4
                        {  9,  9,     -10.5f,       8.5f,       -0.1f,        0.3f },      // 5
                        { 10,  0,      -1.9f,       0.0f,        0.0f,        0.0f },      // 6
                        { 10,  1,      -6.5f,       3.3f,        0.0f,        0.1f },      // 7
                        { 10,  2,       0.2f,      -0.3f,       -0.1f,       -0.1f },      // 8
                        { 10,  3,       0.6f,       4.6f,        0.3f,        0.0f },      // 9
                        { 10,  4,      -0.6f,       4.4f,       -0.1f,        0.0f },      // 60
                        { 10,  5,       1.7f,      -7.9f,       -0.1f,       -0.2f },      // 1
                        { 10,  6,      -0.7f,      -0.6f,       -0.1f,        0.1f },      // 2
                        { 10,  7,       2.1f,      -4.1f,        0.0f,       -0.1f },      // 3
                        { 10,  8,       2.3f,      -2.8f,       -0.2f,       -0.2f },      // 4
                        { 10,  9,      -1.8f,      -1.1f,       -0.1f,        0.1f },      // 5
                        { 10, 10,      -3.6f,      -8.7f,       -0.2f,       -0.1f },      // 6
                        { 11,  0,       3.1f,       0.0f,        0.0f,        0.0f },      // 7
                        { 11,  1,      -1.5f,      -0.1f,        0.0f,        0.0f },      // 8
                        { 11,  2,      -2.3f,       2.1f,       -0.1f,        0.1f },      // 9
                        { 11,  3,       2.1f,      -0.7f,        0.1f,        0.0f },      // 70
                        { 11,  4,      -0.9f,      -1.1f,        0.0f,        0.1f },      // 1
                        { 11,  5,       0.6f,       0.7f,        0.0f,        0.0f },      // 2
                        { 11,  6,      -0.7f,      -0.2f,        0.0f,        0.0f },      // 3
                        { 11,  7,       0.2f,      -2.1f,        0.0f,        0.1f },      // 4
                        { 11,  8,       1.7f,      -1.5f,        0.0f,        0.0f },      // 5
                        { 11,  9,      -0.2f,      -2.5f,        0.0f,       -0.1f },      // 6
                        { 11, 10,       0.4f,      -2.0f,       -0.1f,        0.0f },      // 7
                        { 11, 11,       3.5f,      -2.3f,       -0.1f,       -0.1f },      // 8
                        { 12,  0,      -2.0f,       0.0f,        0.1f,        0.0f },      // 9
                        { 12,  1,      -0.3f,      -1.0f,        0.0f,        0.0f },      // 80
                        { 12,  2,       0.4f,       0.5f,        0.0f,        0.0f },      // 1
                        { 12,  3,       1.3f,       1.8f,        0.1f,       -0.1f },      // 2
                        { 12,  4,      -0.9f,      -2.2f,       -0.1f,        0.0f },      // 3
                        { 12,  5,       0.9f,       0.3f,        0.0f,        0.0f },      // 4
                        { 12,  6,       0.1f,       0.7f,        0.1f,        0.0f },      // 5
                        { 12,  7,       0.5f,      -0.1f,        0.0f,        0.0f },      // 6
                        { 12,  8,      -0.4f,       0.3f,        0.0f,        0.0f },      // 7
                        { 12,  9,      -0.4f,       0.2f,        0.0f,        0.0f },      // 8
                        { 12, 10,       0.2f,      -0.9f,        0.0f,        0.0f },      // 9
                        { 12, 11,      -0.9f,      -0.2f,        0.0f,        0.0f },      // 90
                        { 12, 12,       0.0f,       0.7f,        0.0f,        0.0f } };    // 1

/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART or SPI.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskWorldMagneticModel(void const *argument)
{
    //
#ifdef GPS
    float B[3];
    float decl_deg;
#endif

    WMM_Initialize();

    gWorldMagModel.decl_rad = (real)0.0;
    gWorldMagModel.validSoln = FALSE;
    gWorldMagModel.timeOfLastSoln = 0;

// Can set this in "LG_To_INS_..."
//    gWorldMagModel.timeOfLastSoln = LIMIT_DECL_EXPIRATION_TIME -
//                                    gGpsDataPtr->itow;

    //
    while(1) {
        OS_Delay( 5000 );
#ifdef GPS
        if( gGpsDataPtr->gpsFixType )
        {
            // caution - safety checks on the dates are fine, but we don't want
            // to rely on these hard-coded dates in operation
            // todo - get the GPS date from the external Novatel GPS
            int month = gGpsDataPtr->GPSmonth;
            int day   = gGpsDataPtr->GPSday;
            int year  = gGpsDataPtr->GPSyear;
            if ( year < 16 || year > 20 ) {
                month = 6;
                day = 21;
                year = 17;
            } else if ( day < 1 || day > 31 ) {
                month = 6;
                day = 21;
                year = 17;
            } else if ( month < 1 || month > 12 ) {
                month = 6;
                day = 21;
                year = 17;
            }

            WMM_GetMagVector( (float)gGpsDataPtr->lat,   // [ deg ]
                              (float)gGpsDataPtr->lon,   // [ deg ]
                              (float)(gGpsDataPtr->alt * 0.001),                         // [ km ]
                              month,
                              day,
                              year + 2000,
                              &B[0],                                            // [ nT ]
                              &decl_deg );                                      // [ deg ]
            gWorldMagModel.decl_rad = decl_deg * (real)DEG_TO_RAD;                    // [ rad ]

            gWorldMagModel.validSoln      = TRUE;
            gWorldMagModel.timeOfLastSoln = gGpsDataPtr->itow;
        }
#endif    
    }

}


//void WorldMagneticModel(void)
//{
//    //        WorldMagneticModel;
//    /*
//    % WorldMagneticModel.m
//
//    if( gGpsDataPtr.gpsFixType ),
//    % WMM here
//    declinationAngle_rad = ( 13 + 36 / 60 + 43/3600 ) * CONV.DEG_TO_RAD;
//
//    initLat = gGpsDataPtr.pos(IND.X_AXIS);
//    initLon = gGpsDataPtr.pos(IND.Y_AXIS);
//    initAlt = gGpsDataPtr.pos(IND.Z_AXIS);
//
//    end
//    */
//
//    // VERY SIMPLE WMM (declination near San Jose only)
////    if (gGpsDataPtr->gpsFixType) {
////        gWorldMagModel.decl_rad = (real)( (13.0 + 40.0 / 60.0 + 29.0 / 3600.0) * D2R );
////    } else {
////        gWorldMagModel.decl_rad = (real)0.0;
////    }
//}





/** ****************************************************************************
* @name WMM_Initialize Sets default values for WMM subroutines.
* Trace:
* @param N/A
* @retval
* @brief use - very simple - only two exposed functions
*	WMM_GetMagVector(float Lat, float Lon, float Alt, uint16_t Month, uint16_t
*                   Day, uint16_t Year, float B[3]);
*	e.g. Iceland in may of 2012 = WMM_GetMagVector(65.0, -20.0, 0.0, 5, 5, 2012, B);
*	Alt is above the WGS-84 Ellipsoid
*	B is the NED (XYZ) magnetic vector in nTesla
*	UPDATES : Ellip and MagneticModel
******************************************************************************/
void WMM_Initialize(void)
{
    uint16_t i = 0;

    /// Sets WGS-84 parameters
    Ellip.a = 6378.137f;			///< semi-major axis of the ellipsoid in km
    Ellip.b = 6356.7523142f;		///< semi-minor axis of the ellipsoid in km
    Ellip.fla = 1 / 298.257223563f;	// flattening
    Ellip.eps = (float)sqrt(1 - (Ellip.b*Ellip.b) / (Ellip.a*Ellip.a)); ///< first eccentricity
    Ellip.epssq = (Ellip.eps * Ellip.eps);			///< first eccentricity squared
    Ellip.re = 6371.2f;							///< Earth's radius in km

    /// Sets Magnetic Model parameters
    MagneticModel.nMax       = WMM_MAX_MODEL_DEGREES;
    MagneticModel.nMaxSecVar = WMM_MAX_SECULAR_VARIATION_MODEL_DEGREES;
    MagneticModel.SecularVariationUsed = 0;

    // TODO: Really, Really needs to be read from a file - out of date in 2015 at latest
    MagneticModel.EditionDate = (float)5.7863328170559505e-307;
    MagneticModel.epoch = 2015.0f;
    sprintf(MagneticModel.ModelName, "WMM-2015");

    for (i = 0; i < NUMTERMS; i++){
        MagneticModel.Main_Field_Coeff_G[i]  = coeffs[i][2];
        MagneticModel.Main_Field_Coeff_H[i]  = coeffs[i][3];
        MagneticModel.Secular_Var_Coeff_G[i] = coeffs[i][4];
        MagneticModel.Secular_Var_Coeff_H[i] = coeffs[i][5];
    }
}


/** ****************************************************************************
* @name WMM_GetMagVector
* Trace:
* @param [in] Lat latitude
* @param [in] Lon longitude
* @param [in] AltEllipsoid - height above ellipsoid (km)
* @param [in] Month of the year mm
* @param [in] Day day of month dd
* @param [in] Year yyyy
* @param [out] B pointer to the GeoMagneticElements
* @retval N/A
******************************************************************************/
WMMtype_MagneticModel       TimedMagneticModel;
WMMtype_GeoMagneticElements GeoMagneticElements;
WMMtype_CoordSpherical      CoordSpherical;
WMMtype_CoordGeodetic       CoordGeodetic;
WMMtype_Date                Date;

void WMM_GetMagVector( float    Lat,
                       float    Lon,
                       float    AltEllipsoid,
                       uint16_t Month,
                       uint16_t Day,
                       uint16_t Year,
                       float*    B,
                       float*    wmmDecl )
{
    char Error_Message[255];

    //  WMMtype_MagneticModel  TimedMagneticModel;
    //	WMMtype_CoordSpherical CoordSpherical;
    //	WMMtype_CoordGeodetic  CoordGeodetic;
    //	WMMtype_Date           Date;
    //	WMMtype_GeoMagneticElements GeoMagneticElements;

    CoordGeodetic.lambda = Lon;
    CoordGeodetic.phi    = Lat;
    CoordGeodetic.HeightAboveEllipsoid = AltEllipsoid;
    /// geodetic -> Spherical Equations: 17-18, WMM Technical report
    WMM_GeodeticToSpherical( Ellip,
                             CoordGeodetic,
                             &CoordSpherical );

    Date.Month = Month;
    Date.Day   = Day;
    Date.Year  = Year;
    WMM_DateToYear(&Date, Error_Message);

    /// Time adjust the coefficients, Equation 19, WMM Technical report
    WMM_TimelyModifyMagneticModel( Date,
                                   &MagneticModel,
                                   &TimedMagneticModel );
    /// Computes the geoMagnetic field elements and their time change
    WMM_Geomag( Ellip,
                CoordSpherical,
                CoordGeodetic,
                &TimedMagneticModel,
                &GeoMagneticElements );

    *(B+0) = GeoMagneticElements.X; // North
    *(B+1) = GeoMagneticElements.Y; // East
    *(B+2) = GeoMagneticElements.Z; // Vertical
    // GeoMagneticElements.F is the field strength

    *wmmDecl = GeoMagneticElements.Decl;
}

/** ****************************************************************************
* @name WMM_Geomag API
* @detail
* The main subroutine that calls a sequence of WMM sub-functions to calculate
* the magnetic field elements for a single point. The function expects the
* model coefficients and point coordinates as input and returns the magnetic
* field elements and their rate of change. Though, this subroutine can be
* called successively to calculate a time series, profile or grid of magnetic
* field, these are better achieved by the subroutine WMM_Grid.
* Trace:
* @param [in] Ellip
* @param [in] CoordSpherical
* @param [in] CoordGeodetic
* @param [in] TimedMagneticModel
* @param [out] GeoMagneticElements
* @retval N/A
* @brief CALLS:
* WMM_ComputeSphericalHarmonicVariables( ); Compute Spherical Harmonic variables
* WMM_AssociatedLegendreFunction(); Compute ALF
* WMM_Summation();  Accumulate the spherical harmonic coefficients
* WMM_SecVarSummation(); Sum the Secular Variation Coefficients
* WMM_RotateMagneticVector(); Map the computed Magnetic fields to Geodetic coordinates
* WMM_RotateMagneticVector();  Map the secular variation field components to Geodetic coordinates
* WMM_CalculateGeoMagneticElements(); Geomagnetic elements
* WMM_CalculateSecularVariation() secular variation of each of the Geomagnetic elements
******************************************************************************/
WMMtype_LegendreFunction           LegendreAllocate;
WMMtype_LegendreFunction           *LegendreFunction = &LegendreAllocate;
WMMtype_SphericalHarmonicVariables SphVariables;
WMMtype_MagneticResults            MagneticResultsSph;
WMMtype_MagneticResults            MagneticResultsGeo;
WMMtype_MagneticResults            MagneticResultsSphVar;
WMMtype_MagneticResults            MagneticResultsGeoVar;

uint16_t WMM_Geomag( WMMtype_Ellipsoid           Ellip,
                     WMMtype_CoordSpherical      CoordSpherical,
                     WMMtype_CoordGeodetic       CoordGeodetic,
                     WMMtype_MagneticModel       *TimedMagneticModel,
                     WMMtype_GeoMagneticElements *GeoMagneticElements )
{
    WMM_ComputeSphericalHarmonicVariables( Ellip,
                                           CoordSpherical,
                                           TimedMagneticModel->nMax,
                                           &SphVariables ); ///< Compute Spherical Harmonic variables
    WMM_AssociatedLegendreFunction( CoordSpherical,
                                    TimedMagneticModel->nMax,
                                    LegendreFunction );    ///< Compute ALF
    WMM_Summation( LegendreFunction,
                   TimedMagneticModel,
                   SphVariables,
                   CoordSpherical,
                   &MagneticResultsSph ); ///< Accumulate the spherical harmonic coefficients
    WMM_SecVarSummation( LegendreFunction,
                         TimedMagneticModel,
                         SphVariables,
                         CoordSpherical,
                         &MagneticResultsSphVar ); ///< Sum the Secular Variation Coefficients
    WMM_RotateMagneticVector( CoordSpherical,
                              CoordGeodetic,
                              MagneticResultsSph,
                              &MagneticResultsGeo ); ///< Map the computed Magnetic fields to Geodeitic coordinates
    WMM_RotateMagneticVector( CoordSpherical,
                              CoordGeodetic,
                              MagneticResultsSphVar,
                              &MagneticResultsGeoVar ); ///< Map the secular variation field components to Geodetic coordinates
    WMM_CalculateGeoMagneticElements( &MagneticResultsGeo,
                                      GeoMagneticElements ); ///< Calculate the Geomagnetic elements, Equation 18 , WMM Technical report
    WMM_CalculateSecularVariation( MagneticResultsGeoVar,
                                   GeoMagneticElements ); ///< Calculate the secular variation of each of the Geomagnetic elements

    return TRUE;
}

/** ****************************************************************************
* @name WMM_ComputeSphericalHarmonicVariables Computes Spherical variables
* @detail Variables computed are (a/r)^(n+2), cos_m(lambda) and sin_m(lambda)
*         for spherical harmonic summations. (Equations 10-12 in the WMM
*         Technical Report)
* Trace:
* @param [in] Ellipsoid struct a -   semi-major axis of the ellipsoid
b -   semi-minor axis of the ellipsoid
fla   flattening
epssq first eccentricity squared
eps   first eccentricity
re    mean radius of  ellipsoid
* @param [in] CoordSpherical struct: lambda - longitude
phig - geocentric latitude
r - distance from the center of the ellipsoid
* @param [in] nMax Maximum degree of spherical harmonic secular model
* @param [out] SphVariables Point to data structure with the following elements
* float RelativeRadiusPower[]; [earth_reference_radius_km  sph. radius ]^n
* float cos_mlambda[]; cp(m)  - cosine of (mspherical coord. longitude)
* float sin_mlambda[]; sp(m)  - sine of (mspherical coord. longitude)
* @retval N/A
* @brief CALLS: none
******************************************************************************/
uint16_t WMM_ComputeSphericalHarmonicVariables( WMMtype_Ellipsoid                  Ellip,
                                                WMMtype_CoordSpherical             CoordSpherical,
                                                uint16_t                           nMax,
                                                WMMtype_SphericalHarmonicVariables *SphVariables )
{
    float cos_lambda;
    float sin_lambda;
    uint16_t m;
    uint16_t n;

    //	cos_lambda = (float)cos(DEG2RAD(CoordSpherical.lambda));
    //	sin_lambda = (float)sin(DEG2RAD(CoordSpherical.lambda));
    cos_lambda = (float)cos(DEG_TO_RAD * CoordSpherical.lambda);
    sin_lambda = (float)sin(DEG_TO_RAD * CoordSpherical.lambda);
    /** for n = 0 ... model_order, compute (Radius of Earth / Spherical radius r)^(n+2)
    for n  1..nMax-1 (this is much faster than calling pow MAX_N+1 times).      */
    SphVariables->RelativeRadiusPower[0] = (Ellip.re / CoordSpherical.r) * (Ellip.re / CoordSpherical.r);
    for (n = 1; n <= nMax; n++)	{
        SphVariables->RelativeRadiusPower[n] = SphVariables->RelativeRadiusPower[n - 1] *
            (Ellip.re / CoordSpherical.r);
    }

    /** @brief Compute cos(m*lambda), sin(m*lambda) for m = 0 ... nMax
    cos(a + b) = cos(a)*cos(b) - sin(a)*sin(b)
    sin(a + b) = cos(a)*sin(b) + sin(a)*cos(b) */
    SphVariables->cos_mlambda[0] = 1.0;
    SphVariables->sin_mlambda[0] = 0.0;

    SphVariables->cos_mlambda[1] = cos_lambda;
    SphVariables->sin_mlambda[1] = sin_lambda;
    for (m = 2; m <= nMax; m++)
    {
        SphVariables->cos_mlambda[m] = SphVariables->cos_mlambda[m - 1] * cos_lambda -
                                       SphVariables->sin_mlambda[m - 1] * sin_lambda;
        SphVariables->sin_mlambda[m] = SphVariables->cos_mlambda[m - 1] * sin_lambda +
                                       SphVariables->sin_mlambda[m - 1] * cos_lambda;
    }

    return TRUE;
}  /*WMM_ComputeSphericalHarmonicVariables*/

/** ****************************************************************************
* @name WMM_AssociatedLegendreFunction
* @brief Computes  all of the Schmidt-semi normalized associated Legendre
*        functions up to degree nMax. If nMax <= 16, function WMM_PcupLow is
*        used. Otherwise WMM_PcupHigh is called.
* @param [in] CoordSpherical struct: lambda - longitude
*                                    phig geocentric latitude
*                                    r - distance from the center of the ellipsoid
* @param [in] nMax - Maximum degree of spherical harmonic secular model
* @param [in] LegendreFunction Ptr struct:
*             Pcup - pointer to store Legendre Function  )
*             dPcup - pointer to store  Derivative of Lagendre function
* @param [out] LegendreFunction Calculated Legendre variables in the data structure
* @return always true
******************************************************************************/
uint16_t WMM_AssociatedLegendreFunction( WMMtype_CoordSpherical   CoordSpherical,
                                         uint16_t                 nMax,
                                         WMMtype_LegendreFunction *LegendreFunction )
{
    float    sin_phi;
    uint16_t FLAG = 1;

    //	sin_phi =  (float)sin ( DEG2RAD( CoordSpherical.phig ) ); ///< sin  (geocentric latitude)
    sin_phi = (float)sin(DEG_TO_RAD * CoordSpherical.phig); ///< sin  (geocentric latitude)

    if (nMax <= 16 || (1 - fabs(sin_phi)) < 1.0e-10) ///< If nMax is less than 16 or at the poles
        FLAG = WMM_PcupLow(LegendreFunction->Pcup,
        LegendreFunction->dPcup,
        sin_phi,
        nMax);
    else
        FLAG = WMM_PcupHigh(LegendreFunction->Pcup,
        LegendreFunction->dPcup,
        sin_phi,
        nMax);
    if (FLAG == 0) // Error while computing  Legendre variables
        return FALSE;

    return TRUE;
} /*WMM_AssociatedLegendreFunction */

/** ****************************************************************************
* @name WMM_Summation
* @details Computes Geomagnetic Field Elements X, Y and Z in Spherical coordinate
*         system using spherical harmonic summation.
* The vector Magnetic field is given by -grad V, where V is Geomagnetic
*  scalar potential The gradient in spherical coordinates is given by:
*	<pre>	 dV ^     1 dV ^        1     dV ^
*  grad V = -- r  +  - -- t  +  -------- -- p
*			 dr       r dt       r sin(t) dp               </pre>
* @param [in] LegendreFunction
* @param [in] MagneticModel
* @param [in] SphVariables
* @param [in]  CoordSpherical
* @param [out] MagneticResults
* @return always true
* @brief CALLS : WMM_SummationSpecial
* @author Manoj Nair
* @date June, 2009 Manoj.C.Nair@Noaa.Gov
******************************************************************************/
uint16_t WMM_Summation( WMMtype_LegendreFunction           *LegendreFunction,
                        WMMtype_MagneticModel              *MagneticModel,
                        WMMtype_SphericalHarmonicVariables SphVariables,
                        WMMtype_CoordSpherical             CoordSpherical,
                        WMMtype_MagneticResults            *MagneticResults )
{
    uint16_t m, n, index;
    float cos_phi;

    MagneticResults->Bz = 0.0;
    MagneticResults->By = 0.0;
    MagneticResults->Bx = 0.0;
    for (n = 1; n <= MagneticModel->nMax; n++)	{
        for (m = 0; m <= n; m++) {
            index = (n * (n + 1) / 2 + m);

            /*	<pre>	 nMax  	(n+2) 	  n     m            m           m
            Bz =   -SUM (a/r)   (n+1) SUM  [g cos(m p) + h sin(m p)] P (sin(phi))
            n=1      	      m=0   n            n           n  </pre>*/
            /// Equation 12 in the WMM Technical report.  Derivative with respect to radius.
            MagneticResults->Bz -= SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Main_Field_Coeff_G[index] *
                SphVariables.cos_mlambda[m] +
                MagneticModel->Main_Field_Coeff_H[index] *
                SphVariables.sin_mlambda[m]) *
                (float)(n + 1) *
                LegendreFunction->Pcup[index];

            /*	<pre>1 nMax  (n+2)    n     m            m           m
            By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
            n=1             m=0   n            n           n  </pre>*/
            /* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */
            MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Main_Field_Coeff_G[index] *
                SphVariables.sin_mlambda[m] -
                MagneticModel->Main_Field_Coeff_H[index] *
                SphVariables.cos_mlambda[m]) *
                (float)(m)*
                LegendreFunction->Pcup[index];
            /* <pre>   nMax  (n+2) n     m            m           m
            Bx = - SUM (a/r)   SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
            n=1         m=0   n            n           n  </pre>*/
            /// Equation 10  in the WMM Technical report. Derivative with respect to latitude, divided by radius.

            MagneticResults->Bx -= SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Main_Field_Coeff_G[index] *
                SphVariables.cos_mlambda[m] +
                MagneticModel->Main_Field_Coeff_H[index] *
                SphVariables.sin_mlambda[m]) *
                LegendreFunction->dPcup[index];
        }
    }

    //	cos_phi = (float)cos( DEG2RAD( CoordSpherical.phig ) );
    cos_phi = (float)cos(DEG_TO_RAD * CoordSpherical.phig);
    if (fabs(cos_phi) > 1.0e-10) {
        MagneticResults->By = MagneticResults->By / cos_phi;
    }
    else {
        /** @brief Special calculation for component - By - at Geographic poles.
        * If the user wants to avoid using this function,  please make sure that
        * the latitude is not exactly +/-90. An option is to make use the function
        * WMM_CheckGeographicPoles. */
        WMM_SummationSpecial(MagneticModel,
            SphVariables,
            CoordSpherical,
            MagneticResults);
    }
    return TRUE;
}/*WMM_Summation */

/** ****************************************************************************
* @name WMM_SecVarSummation
* @details This Function sums the secular variation coefficients to get the secular
*          variation of the Magnetic vector.
* @paaram [in] LegendreFunction
* @paaram [in] MagneticModel
* @paaram [in] SphVariables
* @paaram [in] CoordSpherical
* @paaram [out]  MagneticResults
* @return always true
* @brief CALLS : WMM_SecVarSummationSpecial
******************************************************************************/
uint16_t WMM_SecVarSummation( WMMtype_LegendreFunction           *LegendreFunction,
                              WMMtype_MagneticModel              *MagneticModel,
                              WMMtype_SphericalHarmonicVariables SphVariables,
                              WMMtype_CoordSpherical             CoordSpherical,
                              WMMtype_MagneticResults            *MagneticResults )
{
    uint16_t m, n, index;
    float cos_phi;

    MagneticModel->SecularVariationUsed = TRUE;
    MagneticResults->Bz = 0.0;
    MagneticResults->By = 0.0;
    MagneticResults->Bx = 0.0;

    for (n = 1; n <= MagneticModel->nMaxSecVar; n++) {
        for (m = 0; m <= n; m++) {
            index = (n * (n + 1) / 2 + m);

            /**	<pre>	 nMax  	(n+2) 	  n     m            m           m
            Bz =   -SUM (a/r)   (n+1) SUM  [g cos(m p) + h sin(m p)] P (sin(phi))
            n=1      	      m=0   n            n           n  </pre>*/
            /*  Derivative with respect to radius.*/
            MagneticResults->Bz -= SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Secular_Var_Coeff_G[index] *
                SphVariables.cos_mlambda[m] +
                MagneticModel->Secular_Var_Coeff_H[index] *
                SphVariables.sin_mlambda[m]) *
                (float)(n + 1) *
                LegendreFunction->Pcup[index];

            /** <pre> 1 nMax  (n+2)    n     m            m           m
            By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
            n=1             m=0   n            n           n  </pre>*/
            /// Derivative with respect to longitude, divided by radius.
            MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Secular_Var_Coeff_G[index] *
                SphVariables.sin_mlambda[m] -
                MagneticModel->Secular_Var_Coeff_H[index] *
                SphVariables.cos_mlambda[m]) * (float)(m)*
                LegendreFunction->Pcup[index];
            /**	<pre>  nMax  (n+2) n     m            m           m
            Bx = - SUM (a/r)   SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
            n=1         m=0   n            n           n  </pre>*/
            /// Derivative with respect to latitude, divided by radius.
            MagneticResults->Bx -= SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Secular_Var_Coeff_G[index] *
                SphVariables.cos_mlambda[m] +
                MagneticModel->Secular_Var_Coeff_H[index] *
                SphVariables.sin_mlambda[m]) *
                LegendreFunction->dPcup[index];
        }
    }
    //	cos_phi = (float)cos( DEG2RAD( CoordSpherical.phig ) );
    cos_phi = (float)cos(DEG_TO_RAD * CoordSpherical.phig);
    if (fabs(cos_phi) > 1.0e-10) {
        MagneticResults->By = MagneticResults->By / cos_phi;
    }
    else {
        /// Special calculation for component By at Geographic poles
        WMM_SecVarSummationSpecial(MagneticModel,
            SphVariables,
            CoordSpherical,
            MagneticResults);
    }
    return TRUE;
} /*WMM_SecVarSummation*/

/** ****************************************************************************
* @name WMM_RotateMagneticVector
* @details Rotate the Magnetic Vectors to Geodetic Coordinates
* @author Manoj Nair
* @date June, 2009 Manoj.C.Nair@Noaa.Gov
* @details Equation 16, WMM Technical report
* #param [in] CoordSpherical struct WMMtype_CoordSpherical:
*	    lambda - longitude
*		phig - geocentric latitude
*		r  	 - distance from the center of the ellipsoid
* @param [in]	CoordGeodetic struct WMMtype_CoordGeodetic:
*	 lambda - longitude
*	 phi - geodetic latitude
*	 HeightAboveEllipsoid - height above the ellipsoid (HaE)
*   HeightAboveGeoid - height above the Geoid
* @param [in] MagneticResultsSph structure WMMtype_MagneticResults:
*	 Bx - North
*	 By	- East
*	 Bz - Down
* @param[out] MagneticResultsGeo Ptr struct WMMtype_MagneticResults:
*	Bx - North
*  By - East
*  Bz - Down
* @brief CALLS : none
******************************************************************************/
uint16_t WMM_RotateMagneticVector( WMMtype_CoordSpherical  CoordSpherical,
                                   WMMtype_CoordGeodetic   CoordGeodetic,
                                   WMMtype_MagneticResults MagneticResultsSph,
                                   WMMtype_MagneticResults *MagneticResultsGeo )
{
    float  Psi;
    /// Difference between the spherical and Geodetic latitudes
    Psi = (float)DEG_TO_RAD * (CoordSpherical.phig - CoordGeodetic.phi);

    /// Rotate spherical field components to the Geodeitic system
    MagneticResultsGeo->Bz = MagneticResultsSph.Bx * (float)sin(Psi) + MagneticResultsSph.Bz * (float)cos(Psi);
    MagneticResultsGeo->Bx = MagneticResultsSph.Bx * (float)cos(Psi) - MagneticResultsSph.Bz * (float)sin(Psi);
    MagneticResultsGeo->By = MagneticResultsSph.By;

    return TRUE;
}   /*WMM_RotateMagneticVector*/

/** ****************************************************************************
* @name WMM_CalculateGeoMagneticElements
* @brief Calculate all the Geomagnetic elements from X,Y and Z components
* @param [in] MagneticResultsGeo Ptr struct:
*			Bx North
* 			By East
*			Bz Down
* @param [out] GeoMagneticElements Ptr struct:
*	Decl - Angle between the magnetic field vector and true north, + east
* 	Incl - Angle between the magnetic field vector and the horizontal plane, + down
* 	F - Magnetic Field Strength
* 	H - Horizontal Magnetic Field Strength
* 	X - Northern component of the magnetic field vector
* 	Y - Eastern component of the magnetic field vector
*	Z - Downward component of the magnetic field vector
* @brief CALLS : none
******************************************************************************/
uint16_t WMM_CalculateGeoMagneticElements( WMMtype_MagneticResults     *MagneticResultsGeo,
                                           WMMtype_GeoMagneticElements *GeoMagneticElements )
{
    GeoMagneticElements->X = MagneticResultsGeo->Bx;
    GeoMagneticElements->Y = MagneticResultsGeo->By;
    GeoMagneticElements->Z = MagneticResultsGeo->Bz;

    GeoMagneticElements->H = (float)sqrt( MagneticResultsGeo->Bx * MagneticResultsGeo->Bx +
                                          MagneticResultsGeo->By * MagneticResultsGeo->By );
    GeoMagneticElements->F = (float)sqrt( GeoMagneticElements->H * GeoMagneticElements->H +
                                          MagneticResultsGeo->Bz * MagneticResultsGeo->Bz );
    //	GeoMagneticElements->Decl = (float)RAD2DEG(atan2 (GeoMagneticElements->Y , GeoMagneticElements->X));
    //	GeoMagneticElements->Incl = (float)RAD2DEG(atan2 (GeoMagneticElements->Z , GeoMagneticElements->H));
    GeoMagneticElements->Decl = (float)RAD_TO_DEG * atan2(GeoMagneticElements->Y, GeoMagneticElements->X);
    GeoMagneticElements->Incl = (float)RAD_TO_DEG * atan2(GeoMagneticElements->Z, GeoMagneticElements->H);

    return TRUE;
}  /*WMM_CalculateGeoMagneticElements */

/** ****************************************************************************
* @name WMM_CalculateSecularVariation
* @details Takes the Magnetic Variation in x, y, and z and uses it to calculate
*          the secular variation of each of the Geomagnetic elements.
* @param [in] MagneticVariation struct
*				Bx North
*				By East
*				Bz Down
* @param [out]	MagneticElements Ptr struct
*	    Decldot - Yearly Rate of change in declination
* 		Incldot - Yearly Rate of change in inclination
* 		Fdot - Yearly rate of change in Magnetic field strength
* 		Hdot - Yearly rate of change in horizontal field strength
*		Xdot - Yearly rate of change in the northern component
*		Ydot - Yearly rate of change in the eastern component
*		Zdot - Yearly rate of change in the downward component
*		GVdot - Yearly rate of change in grid variation
* @brief CALLS : none
******************************************************************************/
uint16_t WMM_CalculateSecularVariation( WMMtype_MagneticResults     MagneticVariation,
                                        WMMtype_GeoMagneticElements *MagneticElements )
{
    MagneticElements->Xdot = MagneticVariation.Bx;
    MagneticElements->Ydot = MagneticVariation.By;
    MagneticElements->Zdot = MagneticVariation.Bz;
    MagneticElements->Hdot = ( MagneticElements->X * MagneticElements->Xdot +
                               MagneticElements->Y * MagneticElements->Ydot ) / MagneticElements->H; ///< See equation 19 in the WMM technical report
    MagneticElements->Fdot = ( MagneticElements->X * MagneticElements->Xdot +
                               MagneticElements->Y * MagneticElements->Ydot +
                               MagneticElements->Z * MagneticElements->Zdot) / MagneticElements->F;
    MagneticElements->Decldot = (float)RAD_TO_DEG * ( MagneticElements->X * MagneticElements->Ydot -
                                                      MagneticElements->Y * MagneticElements->Xdot ) /
                                                    ( MagneticElements->H * MagneticElements->H );
    MagneticElements->Incldot = (float)RAD_TO_DEG * ( MagneticElements->H * MagneticElements->Zdot -
                                                      MagneticElements->Z * MagneticElements->Hdot ) /
                                                    ( MagneticElements->F * MagneticElements->F );
    MagneticElements->GVdot = MagneticElements->Decldot;

    return TRUE;
} /*WMM_CalculateSecularVariation*/

/** ***************************************************************************
* @brief WMM_PcupHigh
* @details	Evaluates all of the Schmidt-semi normalized associated Legendre
*	functions up to degree nMax. The functions are initially scaled by
*	10^280 sin^m in order to minimize the effects of underflow at large m
*	near the poles (see Holmes and Featherstone 2002, J. Geodesy, 76, 279-299).
*	Note that this function performs the same operation as WMM_PcupLow.
*	However this function also can be used for high degree (large nMax) models.
* param [in] nMax - Maximum spherical harmonic degree to compute.
* @param [in] x - cos(latitude) or sin(latitude).
* @param [out]	Pcup - A vector of all associated Legendgre polynomials evaluated
*              at x up to nMax. The length must by greater or equal to
*              (nMax+1)*(nMax+2)/2.
* @param [out] dPcup - Derivative of Pcup(x) with respect to latitude
* @brief CALLS : none
* @details	Notes:
*  Adopted from the FORTRAN code written by Mark Wieczorek September 25, 2005.
* Manoj Nair, Nov, 2009 Manoj.C.Nair@Noaa.Gov
* Change from the previous version
* The previous version computes the derivatives as
* dP(n,m)(x)/dx, where x = sin(latitude) (or cos(colatitude) ).
* However, the WMM Geomagnetic routines requires dP(n,m)(x)/dlatitude.
* Hence the derivatives are multiplied by sin(latitude).
* Removed the options for CS phase and normalizations.
*
* Note: In geomagnetism, the derivatives of ALF are usually found with
* respect to the colatitudes. Here the derivatives are found with respect
* to the latitude. The difference is a sign reversal for the derivative of
* the Associated Legendre Functions.
*
* The derivatives can't be computed for latitude = |90| degrees.
******************************************************************************/
float f1[NUMPCUP], f2[NUMPCUP], PreSqr[NUMPCUP];

uint16_t WMM_PcupHigh( float    *Pcup,
                       float    *dPcup,
                       float    x,
                       uint16_t nMax )
{
    float  pm2, pm1, pmm, plm, rescalem, z, scalef;
    //	float f1[NUMPCUP], f2[NUMPCUP], PreSqr[NUMPCUP];
    uint16_t k, kstart, m, n;

    if (fabs(x) == 1.0)	{
        // printf("Error in PcupHigh: derivative cannot be calculated at poles\n");
        return FALSE;
    }

    scalef = (float)1.0e-280;

    for (n = 0; n <= 2 * nMax + 1; ++n) {
        PreSqr[n] = (float)sqrt((float)(n));
    }

    k = 2;

    for (n = 2; n <= nMax; n++) {
        k = k + 1;
        f1[k] = (float)(2 * n - 1) / (float)(n);
        f2[k] = (float)(n - 1) / (float)(n);
        for (m = 1; m <= n - 2; m++)	{
            k++;
            f1[k] = (float)(2 * n - 1) / PreSqr[n + m] / PreSqr[n - m];
            f2[k] = PreSqr[n - m - 1] * PreSqr[n + m - 1] / PreSqr[n + m] / PreSqr[n - m];
        }
        k = k + 2;
    }

    /// z = sin (geocentric latitude)
    z = (float)sqrt((1.0f - x) * (1.0f + x));
    pm2 = 1.0f;
    Pcup[0] = 1.0f;
    dPcup[0] = 0.0f;
    if (nMax == 0)
        return FALSE;
    pm1 = x;
    Pcup[1] = pm1;
    dPcup[1] = z;
    k = 1;

    for (n = 2; n <= nMax; n++)	{
        k += n;
        plm = f1[k] * x * pm1 - f2[k] * pm2;
        Pcup[k] = plm;
        dPcup[k] = (float)(n)* (pm1 - x * plm) / z;
        pm2 = pm1;
        pm1 = plm;
    }

    pmm = PreSqr[2] * scalef;
    rescalem = 1.0f / scalef;
    kstart = 0;

    for (m = 1; m <= nMax - 1; ++m) {
        rescalem *= z;

        /// Calculate Pcup(m,m)
        kstart = kstart + m + 1;
        pmm = pmm * PreSqr[2 * m + 1] / PreSqr[2 * m];
        Pcup[kstart] = pmm*rescalem / PreSqr[2 * m + 1];
        dPcup[kstart] = -((float)(m)* x * Pcup[kstart] / z);
        pm2 = pmm / PreSqr[2 * m + 1];
        /// Calculate Pcup(m+1,m)
        k = kstart + m + 1;
        pm1 = x * PreSqr[2 * m + 1] * pm2;
        Pcup[k] = pm1 * rescalem;
        dPcup[k] = ((pm2*rescalem) * PreSqr[2 * m + 1] - x * (float)(m + 1) * Pcup[k]) / z;
        /// Calculate Pcup(n,m)
        for (n = m + 2; n <= nMax; ++n) {
            k += n;
            plm = x * f1[k] * pm1 - f2[k] * pm2;
            Pcup[k] = plm * rescalem;
            dPcup[k] = (PreSqr[n + m] * PreSqr[n - m] * (pm1 * rescalem) - (float)(n)* x * Pcup[k]) / z;
            pm2 = pm1;
            pm1 = plm;
        }
    }

    /// Calculate Pcup(nMax,nMax)
    rescalem = rescalem * z;
    kstart = kstart + m + 1;
    pmm = pmm / PreSqr[2 * nMax];
    Pcup[kstart] = pmm * rescalem;
    dPcup[kstart] = -(float)(nMax)* x * Pcup[kstart] / z;

    return TRUE;
} /* WMM_PcupHigh */

/** ***************************************************************************
* @name WMM_PcupLow
* @brief function evaluates all of the Schmidt-semi normalized associated Legendre
*	      functions up to degree nMax.
* @param [in] nMax -Maximum spherical harmonic degree to compute.
* @param [in] x - cos(colatitude) or sin(latitude).
* @param [out]	Pcup - A vector of all associated Legendgre polynomials
*              evaluated at x up to nMax.
* @param [in] dPcup - Derivative of Pcup(x) with respect to latitude
* @details	Notes: Overflow may occur if nMax > 20 , especially for
*          high-latitudes. Use WMM_PcupHigh for large nMax.
* @author Manoj Nair
* @date June, 2009 . Manoj.C.Nair@Noaa.Gov.
* @brief
*  Note: In geomagnetism, the derivatives of ALF are usually found with
* respect to the colatitudes. Here the derivatives are found with respect
* to the latitude. The difference is a sign reversal for the derivative of
* the Associated Legendre Functions.
******************************************************************************/
float schmidtQuasiNorm[NUMPCUP];

uint16_t WMM_PcupLow( float    *Pcup,
                      float    *dPcup,
                      float    x,
                      uint16_t nMax )
{
    uint16_t n, m, index, index1, index2;
    float k, z;
    //    float schmidtQuasiNorm[NUMPCUP];

    Pcup[0] = 1.0;
    dPcup[0] = 0.0;
    ///sin (geocentric latitude) - sin_phi
    z = (float)sqrt((1.0f - x) * (1.0f + x));

    /// First,	Compute the Gauss-normalized associated Legendre  functions
    for (n = 1; n <= nMax; n++) {
        for (m = 0; m <= n; m++) {
            index = (n * (n + 1) / 2 + m);
            if (n == m)	{
                index1 = (n - 1) * n / 2 + m - 1;
                Pcup[index] = z * Pcup[index1];
                dPcup[index] = z *  dPcup[index1] + x *  Pcup[index1];
            }
            else if (n == 1 && m == 0) {
                index1 = (n - 1) * n / 2 + m;
                Pcup[index] = x *  Pcup[index1];
                dPcup[index] = x *  dPcup[index1] - z *  Pcup[index1];
            }
            else if (n > 1 && n != m) {
                index1 = (n - 2) * (n - 1) / 2 + m;
                index2 = (n - 1) * n / 2 + m;
                if (m > n - 2)	{
                    Pcup[index] = x *  Pcup[index2];
                    dPcup[index] = x *  dPcup[index2] - z *  Pcup[index2];
                }
                else {
                    k = (float)(((n - 1) * (n - 1)) - (m * m)) /
                        (float)((2 * n - 1) * (2 * n - 3));
                    Pcup[index] = x *  Pcup[index2] - k  *  Pcup[index1];
                    dPcup[index] = x *  dPcup[index2] - z *  Pcup[index2] - k *  dPcup[index1];
                }
            }
        }
    }

    /** Compute the ration between the Gauss-normalized associated Legendre
    functions and the Schmidt quasi-normalized version. This is equivalent to
    sqrt((m==0?1:2)*(n-m)!/(n+m!))*(2n-1)!!/(n-m)!  */
    schmidtQuasiNorm[0] = 1.0;
    for (n = 1; n <= nMax; n++)	{
        index = (n * (n + 1) / 2);
        index1 = (n - 1)  * n / 2;
        /* for m = 0 */
        schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * (float)(2 * n - 1) / (float)n;

        for (m = 1; m <= n; m++) {
            index = (n * (n + 1) / 2 + m);
            index1 = (n * (n + 1) / 2 + m - 1);
            schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * (float)sqrt((float)((n - m + 1) * (m == 1 ? 2 : 1)) /
                (float)(n + m));
        }
    }

    /** Converts the  Gauss-normalized associated Legendre
    functions to the Schmidt quasi-normalized version using pre-computed
    relation stored in the variable schmidtQuasiNorm */
    for (n = 1; n <= nMax; n++) {
        for (m = 0; m <= n; m++) {
            index = (n * (n + 1) / 2 + m);
            Pcup[index] = Pcup[index] * schmidtQuasiNorm[index];
            dPcup[index] = -dPcup[index] * schmidtQuasiNorm[index];
            /** The sign is changed since the new WMM routines use derivative
            with respect to latitude instead of co-latitude */
        }
    }

    return TRUE;
}   /*WMM_PcupLow */

/** ***************************************************************************
* @name WMM_SummationSpecial
* @brief Special calculation for the component By at Geographic poles.
* @author Manoj Nair
* @date June, 2009 manoj.c.nair@noaa.gov
* @param [in] MagneticModel
* @param [in] SphVariables
* @param [in] CoordSpherical
* @param [out]  MagneticResults
* @briefCALLS : none
* @details	See Section 1.4, "SINGULARITIES AT THE GEOGRAPHIC POLES", WMM
*  Technical report
******************************************************************************/
float  PcupS[NUMPCUPS];
uint16_t WMM_SummationSpecial( WMMtype_MagneticModel              *MagneticModel,
                               WMMtype_SphericalHarmonicVariables SphVariables,
                               WMMtype_CoordSpherical             CoordSpherical,
                               WMMtype_MagneticResults            *MagneticResults )
{
    uint16_t n, index;
    float k, sin_phi, schmidtQuasiNorm1, schmidtQuasiNorm2, schmidtQuasiNorm3;
    //    float  PcupS[NUMPCUPS];

    PcupS[0] = 1;
    schmidtQuasiNorm1 = 1.0;
    MagneticResults->By = 0.0;
    //	sin_phi = (float)sin( (float)DEG2RAD( CoordSpherical.phig ) );
    sin_phi = (float)sin((float)DEG_TO_RAD * CoordSpherical.phig);

    for (n = 1; n <= MagneticModel->nMax; n++)
    {
        /** Compute the ratio between the Gauss-normalized associated Legendre
        functions and the Schmidt quasi-normalized version. This is
        equivalent to sqrt((m==0?1:2)*(n-m)!/(n+m!))*(2n-1)!!/(n-m)!  */
        index = (n * (n + 1) / 2 + 1);
        schmidtQuasiNorm2 = schmidtQuasiNorm1 * (float)(2 * n - 1) / (float)n;
        schmidtQuasiNorm3 = schmidtQuasiNorm2 *  (float)sqrt((float)(n * 2) / (float)(n + 1));
        schmidtQuasiNorm1 = schmidtQuasiNorm2;
        if (n == 1)	{
            PcupS[n] = PcupS[n - 1];
        }
        else {
            k = (float)(((n - 1) * (n - 1)) - 1) /
                (float)((2 * n - 1) * (2 * n - 3));
            PcupS[n] = sin_phi * PcupS[n - 1] - k * PcupS[n - 2];
        }

        /** <pre> 1 nMax  (n+2)    n     m            m           m
        By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
        n=1             m=0   n            n           n  </pre>*/
        /* Equation 11 in the WMM Technical report. Derivative with respect to
        longitude, divided by radius. */
        MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
            (MagneticModel->Main_Field_Coeff_G[index] *
            SphVariables.sin_mlambda[1] -
            MagneticModel->Main_Field_Coeff_H[index] *
            SphVariables.cos_mlambda[1]) *
            PcupS[n] *
            schmidtQuasiNorm3;
    }
    return TRUE;
}/*WMM_SummationSpecial */

/** ***************************************************************************
* @name WMM_SecVarSummationSpecial
* @brief Special calculation for the secular variation summation at the poles.
* @param [in] MagneticModel
* @param [in] SphVariables
* @param [in] CoordSpherical
* @param [out] MagneticResults
* @brief CALLS : none
******************************************************************************/
//float PcupS[NUMPCUPS];
uint16_t WMM_SecVarSummationSpecial( WMMtype_MagneticModel              *MagneticModel,
                                     WMMtype_SphericalHarmonicVariables SphVariables,
                                     WMMtype_CoordSpherical             CoordSpherical,
                                     WMMtype_MagneticResults            *MagneticResults )
{
    uint16_t n, index;
    float k, sin_phi, schmidtQuasiNorm1, schmidtQuasiNorm2, schmidtQuasiNorm3;
    //    float PcupS[NUMPCUPS];

    PcupS[0] = 1;
    schmidtQuasiNorm1 = 1.0;
    MagneticResults->By = 0.0;
    //	sin_phi = (float)sin( (float)DEG2RAD( CoordSpherical.phig ) );
    sin_phi = (float)sin((float)DEG_TO_RAD * CoordSpherical.phig);

    for (n = 1; n <= MagneticModel->nMaxSecVar; n++) {
        index = (n * (n + 1) / 2 + 1);
        schmidtQuasiNorm2 = schmidtQuasiNorm1 * (float)(2 * n - 1) / (float)n;
        schmidtQuasiNorm3 = schmidtQuasiNorm2 *  (float)sqrt((float)(n * 2) / (float)(n + 1));
        schmidtQuasiNorm1 = schmidtQuasiNorm2;
        if (n == 1)	{
            PcupS[n] = PcupS[n - 1];
        }
        else {
            k = (float)(((n - 1) * (n - 1)) - 1) /
                (float)((2 * n - 1) * (2 * n - 3));
            PcupS[n] = sin_phi * PcupS[n - 1] - k * PcupS[n - 2];
        }

        /**<pre>  1 nMax  (n+2)    n     m            m           m
        By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
        n=1             m=0   n            n           n  </pre>*/
        /// Derivative with respect to longitude, divided by radius.
        MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
            (MagneticModel->Secular_Var_Coeff_G[index] *
            SphVariables.sin_mlambda[1] -
            MagneticModel->Secular_Var_Coeff_H[index] *
            SphVariables.cos_mlambda[1]) *
            PcupS[n] *
            schmidtQuasiNorm3;
    }
    return TRUE;
}/*SecVarSummationSpecial*/


/** ***************************************************************************
* @name WMM_TimelyModifyMagneticModel
* @details Time change the Model coefficients from the base year of the model using
* secular variation coefficients. Store the coefficients of the static model
* with their values advanced from epoch t0 to epoch t. Copy the SV
* coefficients.  If input "t" is the same as "t0", then this is merely a copy
* operation. If the address of "TimedMagneticModel" is the same as the address
* of "MagneticModel", then this procedure overwrites the given item "MagneticModel".
* @param [in] UserDate
* @param [in] MagneticModel
* @param [out] TimedMagneticModel
* @return N/A
* @brief CALLS : none
******************************************************************************/
void WMM_TimelyModifyMagneticModel( WMMtype_Date          UserDate,
                                    WMMtype_MagneticModel *MagneticModel,
                                    WMMtype_MagneticModel *TimedMagneticModel )
{
    uint16_t n, m, index, a, b;

    TimedMagneticModel->EditionDate = MagneticModel->EditionDate;
    TimedMagneticModel->epoch = MagneticModel->epoch;
    TimedMagneticModel->nMax = MagneticModel->nMax;
    TimedMagneticModel->nMaxSecVar = MagneticModel->nMaxSecVar;
    a = TimedMagneticModel->nMaxSecVar;
    b = (a * (a + 1) / 2 + a);
    strcpy(TimedMagneticModel->ModelName, MagneticModel->ModelName);
    for (n = 1; n <= MagneticModel->nMax; n++)	{
        for (m = 0; m <= n; m++) {
            index = (n * (n + 1) / 2 + m);
            if (index <= b) {
                TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index] +
                    (UserDate.DecimalYear - MagneticModel->epoch) *
                    MagneticModel->Secular_Var_Coeff_H[index];
                TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index] +
                    (UserDate.DecimalYear - MagneticModel->epoch) *
                    MagneticModel->Secular_Var_Coeff_G[index];
                TimedMagneticModel->Secular_Var_Coeff_H[index] = MagneticModel->Secular_Var_Coeff_H[index]; // We need a copy of the secular var coeff to calculate secular change
                TimedMagneticModel->Secular_Var_Coeff_G[index] = MagneticModel->Secular_Var_Coeff_G[index];
            }
            else {
                TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index];
                TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index];
            }
        }
    }
} /* WMM_TimelyModifyMagneticModel */

/** ***************************************************************************
* @name WMM_DateToYear
* @details  Converts a given calendar date into a decimal year
* @param [out] CalendarDate
* @param [out] Error
* @param [out] TimedMagneticModel
* @return always 1
* @brief CALLS : none
******************************************************************************/
uint16_t MonthDays[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

uint16_t WMM_DateToYear( WMMtype_Date *CalendarDate,
                         char         *Error )
{
    uint16_t temp = 0;    /// Total number of days
    uint16_t ExtraDay = 0;
    uint16_t i;

    if ( (CalendarDate->Year % 4 == 0 && CalendarDate->Year % 100 != 0) ||
         CalendarDate->Year % 400 == 0)
    {
        ExtraDay = 1;
    } else {
        ExtraDay = 0;
    }
    MonthDays[2] += ExtraDay;

    /******************Validation********************************/
    if (CalendarDate->Month <= 0 || CalendarDate->Month > 12)
    {
        strcpy(Error, "\nError: The Month entered is invalid, valid months are '1 to 12'\n");
        return 0;
    }

    if (CalendarDate->Day <= 0 || CalendarDate->Day > MonthDays[CalendarDate->Month])
    {
        // printf("\nThe number of days in month %d is %d\n", CalendarDate->Month, MonthDays[CalendarDate->Month]);
        strcpy(Error, "\nError: The day entered is invalid\n");
        return 0;
    }
    /****************Calculation of t***************************/
    for (i = 1; i <= CalendarDate->Month; i++) {
        temp += MonthDays[i - 1];
    }
    temp += CalendarDate->Day;
    CalendarDate->DecimalYear = CalendarDate->Year + (temp - 1) / (365.0f + ExtraDay);

    return 1;
}  /*WMM_DateToYear*/

/** ***************************************************************************
* @name WMM_GeodeticToSpherical
* @details  Converts Geodetic coordinates to Spherical coordinates Convert
* geodetic coordinates, (defined by the WGS-84 reference ellipsoid), to Earth
* Centred Earth Fixed Cartesian coordinates, and then to spherical coordinates.
* @param [in] Ellip
* @param [in] CoordGeodetic
* @param [out] CoordSpherical
* @return always 1
* @brief CALLS : none
******************************************************************************/
void WMM_GeodeticToSpherical( WMMtype_Ellipsoid      Ellip,
                              WMMtype_CoordGeodetic  CoordGeodetic,
                              WMMtype_CoordSpherical *CoordSpherical )
{
    float CosLat, SinLat, rc, xp, zp; ///< all local variables

    //	CosLat = (float)cos( (float)DEG2RAD(CoordGeodetic.phi) );
    //	SinLat = (float)sin( (float)DEG2RAD(CoordGeodetic.phi) );
    CosLat = (float)cos((float)DEG_TO_RAD * CoordGeodetic.phi);
    SinLat = (float)sin((float)DEG_TO_RAD * CoordGeodetic.phi);

    /// compute the local radius of curvature on the WGS-84 reference ellipsoid
    rc = Ellip.a / (float)sqrt(1.0f - Ellip.epssq * SinLat * SinLat);  // [ km ]

    /// compute ECEF Cartesian coordinates of specified point (for longitude=0)
    xp = (rc + CoordGeodetic.HeightAboveEllipsoid) * CosLat;
    zp = (rc * (1.0f - Ellip.epssq) + CoordGeodetic.HeightAboveEllipsoid) * SinLat;

    /// compute spherical radius and angle lambda and phi of specified point
    CoordSpherical->r = (float)sqrt(xp*xp + zp*zp);
    //	CoordSpherical->phig   = (float)RAD2DEG(asin(zp / CoordSpherical->r)); ///< geocentric latitude
    CoordSpherical->phig = (float)RAD_TO_DEG * asin(zp / CoordSpherical->r); ///< geocentric latitude
    CoordSpherical->lambda = CoordGeodetic.lambda;                  ///< longitude
}// WMM_GeodeticToSpherical
