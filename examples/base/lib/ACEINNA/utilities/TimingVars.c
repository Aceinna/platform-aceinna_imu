/*
 * File:   TimingVars.c
 * Author: joemotyka
 *
 * Created on April 9, 2016, 9:39 PM
 */


#include "TimingVars.h"
#include "ucb_packet.h" // UcbGetSysType(); and UNAIDED_AHRS_SYS

#ifdef INS_OFFLINE
#include "c:\Projects\software\sim\INS380_Offline\INS380_Offline\SimulationParameters.h"
#endif

void TimingVars_Increment(void)
{
    // IncrementTimingVars.m
    //
    // Purpose: Increment timing variables that control the operation
    //          of the Extended Kalman Filter (in particular, the
    //          update stage of the EKF).
    //
    // Output: secondCntr -- increments once per secondCntr (1 Hz); not reset.
    //         tenHertzCntr -- increments ten times per second (10 Hz); reset once
    //                         it reached 10.
    //         subFrameCntr -- increments one hundred times per second (100 Hz);
    //                         reset once it reaches 10.
    //         t -- time generated from the three counters
    //

    if (timer.odr == ODR_100_HZ) {
        timer.oneHundredHertzFlag = 1;
    } else {
        timer.oneHundredHertzFlag++;
        if (timer.oneHundredHertzFlag > 1) {
            timer.oneHundredHertzFlag = 0;
        }
    }

    if ( timer.basicFrameCounter >= timer.odr ) {
        timer.basicFrameCounter = 0;
    } else {
        timer.basicFrameCounter++;
    }

    if (timer.oneHundredHertzFlag == 1) {
        timer.subFrameCntr = timer.subFrameCntr + 1;
        if (timer.subFrameCntr >= 10) {
            timer.subFrameCntr = 0;

            timer.tenHertzCntr = timer.tenHertzCntr + 1;
            if (timer.tenHertzCntr >= 10) {
                timer.tenHertzCntr = 0;

                timer.secondCntr = timer.secondCntr + 1;
            }
        }
    }
}

float TimingVars_GetTime(void)
{
    timer.time = (float)( 1.00*timer.secondCntr +
                          0.10*timer.tenHertzCntr +
                          0.01*timer.subFrameCntr);
    //timing.t = timing.t + timing.tMin;

    return(timer.time);
}

void TimingVars_SetTMin(float tMin)
{
    timer.tMin = tMin;
}

float TimingVars_GetTMin(void)
{
    return(timer.tMin);
}

#ifdef DISPLAY_DIAGNOSTIC_MSG
void TimingVars_DisplayTimerVars(signed long timeStep)
{
    std::cout << "Iter " << timeStep << ": " << timer.secondCntr << ", "
                                             << timer.tenHertzCntr << ", "
                                             << timer.subFrameCntr << ", (t = "
                                             << TimingVars_GetTime() << ")\n";
}

void TimingVars_DiagnosticMsg( std::string msg )
{
    std::cout << msg << " (t = " << TimingVars_GetTime()
                     << ", k = " << TimingVars_GetTimeStep()
                     << ")\n";
}
#endif

uint32_t TimingVars_GetTimeStep(void)
{
    return( 100 * timer.secondCntr +
             10 * timer.tenHertzCntr +
              1 * timer.subFrameCntr );
}

void Initialize_Timing(void)
{
    // InitTimingVars.m
    //
    // Purpose: Initialize the timing variables that control operation of the
    //          Extended Kalman Filter
    //
    // Output: secondCntr -- increments once per simulated second (1 Hz); not reset.
    //         tenHertzCntr -- increments ten times per second (10 Hz); reset
    //                         once it reached 10.
    //         subFrameCntr -- increments one hundred times per second (100 Hz);
    //                         reset once it reaches 10.
    //         time -- time generated from the three counters
    //

    // Initialize timing variables
    timer.secondCntr = 0;
    timer.tenHertzCntr = 0;
    timer.subFrameCntr = -1;
    timer.basicFrameCounter = 0; // increments at 100 or 200 Hz and reset after 1 second

    timer.time = 0.0;
    timer.tMin = 0.0;

    // toggles between 0 and 1 at 200 Hz (currently used in firmware)
    timer.oneHundredHertzFlag = 0;

#ifdef INS_OFFLINE
    // This value is set based on the version string specified in the 
    //   simulation configuration file, ekfSim.cfg
    // IMU: 0/1, VG: 2, AHRS: 3, Aided-VG: 4, Aided-AHRS: 5, INS: 6
    uint8_t sysType = gSimulation.sysType;
#else
    // This value is set based on the version string loaded into the unit
    //   via the system configuration load
    uint8_t sysType = UcbGetSysType(); // from system config
#endif

    // Set the execution rate of taskDataAcquisition() based on the system type
    if(sysType <= UNAIDED_AHRS_SYS) {
        // Configured as an IMU or Unaided AHRS/VG: ODR = 200 Hz
        timer.odr = ODR_200_HZ;
    } else {
        // Configured as an Aided-AHRS/VG or INS: ODR = 100 Hz
        timer.odr = ODR_100_HZ;
    }
}

