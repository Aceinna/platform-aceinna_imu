/*
 * File:   EKF_Algorithms.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#include "GlobalConstants.h"   // TRUE, FALSE, etc

#include <math.h>      // std::abs
#include <stdlib.h>   // EXIT_FAILURE

#include "xbowsp_algorithm.h"        // gAlgorithm
#include "xbowsp_generaldrivers.h"   // gCalibration

#include "Indices.h"     // IND

#include "SelectState.h"
#include "EKF_Algorithm.h"
#include "PredictFunctions.h"
#include "UpdateFunctions.h"
#include "AlgorithmLimits.h"
#include "TimingVars.h"

KalmanFilterStruct gKalmanFilter;

// This routine is called at either 100 or 200 Hz (based upon the system
//   configuration):
//      -- Unaided soln: 200 Hz
//      -- Aided soln: 100 Hz
void EKF_Algorithm(void)
{
    static uint16_t freeIntegrationCounter = 0;

    // Compute the EKF solution if past the stabilization and initialization
    //   stages
    if( gAlgorithm.state > INITIALIZE_ATTITUDE )
    {
        // Increment the algorithm itow
        gAlgorithm.itow = gAlgorithm.itow + gAlgorithm.dITOW;

        // Perform EKF Prediction
        EKF_PredictionStage();

        // Update the predicted states if not freely integrating (NOTE: free-
        //   integration is not applicable in HG AHRS mode)
        if( gConfiguration.userBehavior.bit.freeIntegrate &&
            ( gAlgorithm.state > HIGH_GAIN_AHRS ) )
        {
            // Limit the free-integration time before reverting to the complete
            //   EKF solution (including updates)
            freeIntegrationCounter = freeIntegrationCounter + 1;   // [cycles]
            if( freeIntegrationCounter >= gAlgorithm.Limit.Free_Integration_Cntr ) {
                freeIntegrationCounter = 0;
                gConfiguration.userBehavior.bit.freeIntegrate = FALSE;

#ifdef DISPLAY_DIAGNOSTIC_MSG
                // Display the time at the end of the free-integration period
                TimingVars_DiagnosticMsg("Free integration period ended");
#endif
            }

            // Restart the system in LG AHRS after free integration is complete
            gAlgorithm.insFirstTime = TRUE;
            gAlgorithm.state        = LOW_GAIN_AHRS;
            gAlgorithm.stateTimer   = gAlgorithm.Duration.Low_Gain_AHRS;
        } else {
            gConfiguration.userBehavior.bit.freeIntegrate = FALSE;
            freeIntegrationCounter = 0;

            // Perform EKF Update
            EKF_UpdateStage();
        }
    }

    // Select the algorithm state based upon the present state as well as
    //   operational conditions (time, sensor health, etc).  Note: This is called
    //   after the the above code-block to prevent the transition from occuring
    //   until the next time step.
    switch( gAlgorithm.state ) {
        case STABILIZE_SYSTEM:
            StabilizeSystem();
            break;
        case INITIALIZE_ATTITUDE:
            InitializeAttitude();
            break;
        case HIGH_GAIN_AHRS:
            HG_To_LG_Transition_Test();
            break;
        case LOW_GAIN_AHRS:
            // Prevent INS transition by setting
            //   gConiguration.userBehavior.bit.useGPS false (use in simulation
            //   in lieu of AHRS_ONLY)
            LG_To_INS_Transition_Test();
            break;
        case INS_SOLUTION:
            INS_To_AHRS_Transition_Test();
            break;
        default:
#ifdef DISPLAY_DIAGNOSTIC_MSG
            // Shouldn't be able to make it here
            TimingVars_DiagnosticMsg("Uh-oh! Invalid algorithm state in EKF_Algorithm.cpp");
            std::cout << "Press enter to finish ...";
            std::cin.get();
#endif
            return;
    }

    // Dynamic motion logic (to revert back to HG AHRS)
    DynamicMotion();
}

