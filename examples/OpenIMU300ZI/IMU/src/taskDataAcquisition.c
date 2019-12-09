/*****************************************************************************
 * @file   taskDataAcquisition.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 100Hz, gets the data for each sensor
 * and applies available calibration
 ******************************************************************************/
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

#include "algorithmAPI.h"
#include "bitAPI.h"
#include "boardAPI.h"
#include "magAPI.h"
#include "platformAPI.h"
#include "userAPI.h"
#include "spiAPI.h"
#include "taskDataAcquisition.h"
#include "commAPI.h"
#include "UserMessagingSPI.h"

#include "osapi.h"
#include "osresources.h"

/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART or SPI.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskDataAcquisition(void const *argument)
{
    //
    int  res;
    int  spiRateRef = 0, spiRateDiv = 0;

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    BOOL overRange  = FALSE;        //uncomment this line if overrange processing required
#pragma GCC diagnostic warning "-Wunused-but-set-variable"

    // This routine sets up the timer. Can use the structure below this point.
    TaskDataAcquisition_Init();
    // Initialize user algorithm parameters, if needed
    initUserDataProcessingEngine();
    // set the mag align scale factors
    InitMagAlignParams();   
    // Start sensors data acquisition 
    DataAquisitionStart();

    //**************  Add user initialization here, if needed ****************
    //   Data is provided by the primary sensors (accelerometer and rate-sensor)
    //   as fast as possible.  Data is obtained from secondary sensors at a
    //   lower rate.  The rate-sensor data read is synced to TIM5 or an
    //   external signal.  When the read is commanded and in the data buffer,
    //   The data-event flag is set and the wait is bypassed.  Data is obtained
    //   from the buffer, calibrated, filtered and provided to the user for
    //   subsequent processing.

    while( 1 )
    {
        // *****************************************************************
        // NOTE: This task loop runs at 200 Hz
        // *****************************************************************

        PrepareToNewDacqTick();
        
        //  Wait for next tick 
        //  Upon timeout of TIM2 (or user sync), let the process continue
        res = osSemaphoreWait(dataAcqSem, 1000);
        if(res != osOK){
            // Wait timeout expired. Something wrong wit the dacq system
            // Process timeout here
        }
        
        // inform user, that new data set is being prepared (if required)
        // in case of SPI communication interface sets pin DRDY high
        // in case of UART communication interface sets pin IO2 high
        if(platformGetUnitCommunicationType() != UART_COMM){
            setDataReadyPin(1);
        }else{
        setIO2Pin (1);
        }
    

        // Get calibrated sensor data:
        //   Inside this function the sensor data is filtered by a second-order low-pass
        //   Butterworth filter, with a cutoff frequency selected by the user (or zero to
        //   disable).  
        //   Refer to UserConfiguration.c for implementation and to the enumerator structure 
        //   'eFilterType' in file filter.h for available selections.
        //
        //   Low pass filtering is followed by application of the unit calibration parameters
        //   - scaling, temperature compensation, bias removal, and misalignment.
        //
        //   Results are placed in the structure of type double. The pointer to this
        //   structure is 'pScaledSensors'.  Ordering of sensors data in this structure
        //   defined by 'rawSensor_e' enumerator in the file indices.h
        GetSensorsData();

        // Check if sensors data over range
        // If overRange is TRUE - limit values are put into sensors data based 
        // on chosen sensors sensitivity 
        overRange = handleOverRange();          

        // *****************************************************************
        // At this point sensors data is in next units
        // Acceleration - g's 
        // Rates        - rad/s
        // Magnetometer - Gauss
        // Temperature  - degrees C
        //******************************************************************
        
        // Update BIT status
        updateBITandStatus();

        // **********************  Algorithm ************************
        //  Next function get called at 200 Hz
        //  Incorporate timing logic inside algorithm function, if
        //  there is performance inpact on unit due to algorithm complexity
        inertialAndPositionDataProcessing(200);
        
        // Uncomment next line if there is intention of using S0 or S1 legacy
        // packets for continuous data output
        //*****************************************************************
        //applyNewScaledSensorsData();
        //*****************************************************************

        if(platformHasMag() ) {
            // Mag Alignment (follows Kalman filter or user algorithm as the
            // innovation routine calculates the euler angles and the magnetic
            // vector in the NED-frame)
            MagAlign();   // only does this on align
        }

        if(platformGetUnitCommunicationType() != UART_COMM){
            // Perform interface - specific processing here
            FillSPIBurstDataBuffer();
            if(spiRateRef){
                spiRateDiv++;
                if(spiRateDiv >= spiRateRef){
                    // Inform user, that new data set is ready (if required)
                    setDataReadyPin(0); // activate data ready - set low 
                    spiRateDiv = 0;
                    spiRateRef = GetSpiPacketRateDivider();
                }
            }else {
                spiRateRef = GetSpiPacketRateDivider();
            }
            UpdateSpiUserConfig();
        }else {
            // Process user commands and  output continuous packets to UART
            // Processing of user commands always goes first
            // Inform user, that new data set is ready (if required)
            setIO2Pin (0);
            ProcessUserCommands ();
            SendContinuousPacket(200);
        }
    }
}
