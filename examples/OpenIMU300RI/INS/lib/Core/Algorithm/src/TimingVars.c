/******************************************************************************
 * File:   TimingVars.c
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


#include "TimingVars.h"

TimingVars           timer;   // for InitTimingVars

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

    if(timer.dacqFrequency == DACQ_100_HZ){
        timer.oneHundredHertzFlag = 1;
    }else{
        timer.oneHundredHertzFlag++;
        timer.oneHundredHertzFlag &= 1;     // toggle 1Hz flag
    }

    if ( timer.basicFrameCounter >= timer.dacqFrequency) {
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
#ifdef INS_OFFLINE
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
#else
void TimingVars_DisplayTimerVars(signed long timeStep)
{
    DEBUG_INT("Iter", timeStep);
    DEBUG_INT(",", timer.secondCntr);
    DEBUG_INT(",", timer.tenHertzCntr);
    DEBUG_INT(",", timer.subFrameCntr);
    DEBUG_INT(",(", TimingVars_GetTime());
    DEBUG_STRING(")\r\n,");
}

void TimingVars_DiagnosticMsg( char *msg )
{
    DEBUG_STRING(msg);
    DEBUG_INT(" (t = ", TimingVars_GetTime());
    DEBUG_INT(", k = ", TimingVars_GetTimeStep());
    DEBUG_STRING(")\r\n");
}
#endif
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

    // Override execution rate of taskDataAcquisition() based on the configuration
    timer.dacqFrequency = DACQ_200_HZ;     // default
}

