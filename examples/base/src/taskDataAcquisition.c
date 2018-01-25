/** ***************************************************************************
 * @file   taskDataAcquisition.c
 * @Author
 * @date   September, 2008
 * @copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 100Hz, gets the data for each sensor
 * and applies available calibration
 ******************************************************************************/
#include "salvodefs.h"
#include "stm32f2xx.h"
#include "stm32f2xx_conf.h"
#include "bsp.h"
#include "dmu.h"
#include "debug_usart.h"
#include "debug.h"
#include "taskDataAcquisition.h"
#include "temperatureSensor.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "Indices.h"
#include "gyroscope.h"
#include "GpsData.h"

#include "calibrate.h"
#include "filter.h"
#include "xbowsp_generaldrivers.h"
#include "xbowsp_init.h"
#include "UserCommunication_SPI.h"
#include "ucb_packet.h" // UcbGetSysType();
#include "boardDefinition.h" ///< For ONE_PPS_PIN, etc
#include "taskUserCommunication.h"
#include "xbowsp_algorithm.h" ///< EEPROM Lock
#include "sensor_cal.h" // InitMagAlignParams()

#include "WorldMagneticModel.h"

#include "watchdog.h"
#include "bit.h"
#include "bitSelfTest.h"

#include "ucb_packet.h"

#include "EKF_Algorithm.h"
#include <math.h>

#include "AlgorithmLimits.h"
#include "BIT.h"

//#include "CompilerFlags.h"   // for GYRO_MAXIM21000 and GYRO_BMI160

// Create inline macros to replace the commands used to toggle the data-ready line
/// commands to toggle the data-ready line
#define _DataReadyPin_SetHigh(); DATA_READY_PORT->BSRRL = DATA_READY_PIN;
#define _DataReadyPin_SetLow();  DATA_READY_PORT->BSRRH = DATA_READY_PIN;

UcbPacketStruct gSpiUcbPacket; // spi.c needs this for MISO xmit isr

void _TaskDataAcquisition_Init(void);
void _InitExternalSync( FunctionalState NewState );
void _InitSensors( void );

void _ConvertToXBowScaling(int16_t* reading);
uint8_t _Temp_MovingAverage(int16_t* reading);

// todo tm20160603 - PUT IN A BETTER PLACE!  TEMPORARY TO GET THIS COMPILING AND RUNNING!!!
#include "TimingVars.h"
TimingVars           timer;   // for InitTimingVars

#include "MagAlign.h"
MagAlignStruct             gMagAlign;

#include "WorldMagneticModel.h"
WorldMagModelStruct  gWorldMagModel;

#ifdef RUN_PROFILING
uint32_t gEkfElapsedTime;
uint32_t gEkfMaxTime;
float gEkfAvgTime;
#endif

// Hold the sensor readings from the buffer
static int16_t reading[NUM_SENSOR_READINGS] = {0}; ///< raw sensor readings
uint32_t can_bus_heart_beat = 0;
uint32_t tim5_heart_beat = 0;


int calibrateTableValid = 0;
uint32_t ekfCounter = 0;

static inline uint8_t isCalibrateTableValid()
{
  return (calibrateTableValid == 1);
}


/** ***************************************************************************
 * @name _InitExternalSync() LOCAL sets up the sync timer
 * @brief  perform input capture on a clock signal using TIM2
 *
 * @param [in] NewState - interpt enable or disable
 * @retval N/A
 ******************************************************************************/
void _InitExternalSync( FunctionalState NewState )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /// Enable the peripheral clocks
    RCC_AHB1PeriphClockCmd( ONE_PPS_CLK, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG, ENABLE ); // for interrupts

    /// Configure the one-PPS pin (A0) as input
    GPIO_StructInit( &GPIO_InitStructure );
    GPIO_InitStructure.GPIO_Pin  = ONE_PPS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init( ONE_PPS_PORT, &GPIO_InitStructure );

    /// Configure EXTI line
    EXTI_StructInit( &EXTI_InitStructure );
    EXTI_InitStructure.EXTI_Line    = ONE_PPS_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;

    // Check for IMU and GPS message type.  Set the external interrupt
    //   appropriately.
    if( UcbGetSysType() <= UNAIDED_AHRS_SYS ) {
        // IMU/Unaided: look for rising edge of synchronization signal
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    } else {
        // Aided: Configure the synchronization/1-PPS signal as needed
        if ( (gConfiguration.protocolGPS == NOVATEL_BINARY) ||
             (gConfiguration.protocolGPS == NMEA_TEXT)      )
        {
            // For NovAtel, the pulse is 1 msec wide; DRDY 10 msec after edge
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        } else { // SiRf (Origin GPS chip)
            // Pulse is ~200 msec wide; DRDY 300 msec after edge
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        }
    }

    EXTI_InitStructure.EXTI_LineCmd = NewState;
    EXTI_Init( &EXTI_InitStructure );

    // The following variable is unused.  Set appropriately for use later.
    //   FIXME: JSM
    if( UcbGetSysType() <= UNAIDED_AHRS_SYS ) {
        gAlgorithm.bitStatus.hwStatus.bit.unlocked1PPS = TRUE;
    } else {
        gAlgorithm.bitStatus.hwStatus.bit.unlocked1PPS = NewState;
    }

    /// Connect EXTI Line to GPIO Pin
    SYSCFG_EXTILineConfig( ONE_EXTI_PORT_SOURCE, ONE_PPS_EXTI_PIN_SOURCE );

    /// Enable and set EXTI Interrupt to the highest priority
    NVIC_InitStructure.NVIC_IRQChannel                   = ONE_PPS_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = NewState;
    NVIC_Init( &NVIC_InitStructure );
}


static uint8_t  initPpsSync     =  0; // 0 not synced, 1 synced
uint8_t getPpsFlag( void ) { return initPpsSync; } // dmu.h
void    setPpsFlag( uint8_t gotPpsFlag ) { initPpsSync = gotPpsFlag; }

/** ***************************************************************************
 * @name ONE_PPS_EXTI_IRQHandler() LOCAL TIM5 global interrupt request handler.
 * @brief The value of the timer that triggers the interrupt is based on the
 *        input to 'InitDataAcquisitionTimer()'. Upon TIM5 interrupt, the rate
 *        sensor data buffers are reset when the function GyroStartReading() is
 *        called. The sensor data-ready interrupt starts the data transfer to
 *        the data buffers.
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void ONE_PPS_EXTI_IRQHandler(void)
{
    static uint8_t  initSync       = 1;
    static uint8_t  initSpiCntr    = 0;

    static uint8_t  cycleCounter   = 0;
    static uint8_t  N              = 6;
    static uint16_t PeriodsToAverage = 64;   // 2^6 = 64 (make a #define)
    static uint32_t syncCounter    = 0;
    static uint32_t arrVal;
    static uint32_t arrPrev;
    static uint32_t timerCount     = 0;  // is always greater than zero
    static uint32_t timerCountPrev = 0;  // is always greater than zero
    static int32_t  sumDeltaCount  = 0;  // can be + or -

    static uint8_t  ticksRequiredForOutput;

    //  ONE_PPS INTERRUPT HERE
    if( gAlgorithm.algorithmType == INS_ALGO ) {
        // Detected a pulse.  Adjust the timer so the GPS update is applied at
        //   the appropriate moment.  THIS IS NOT HOW THE ONE-PPS WORKS.  SYNC
        //   WITH THE DATA IN TASK-GPS (WHEN NEW DATA ARRIVES)

        if ( IsInternalGPS() == true )  // hasGPS
        {
            // From the Origin GPS datasheet: The correspondent UTC time message
            //   is generated and put into output FIFO 300ms after the PPS
            //   signal. The exact time between PPS and UTC time message
            //   delivery depends on message rate, message queue, and
            //   communication baud rate.
            ;
setPpsFlag(true);
        } else {
            ;
        }
    } else {
        // Not Using GPS for information.  Possible to sync to 1kHz signal.
        if(initSync) {
            // Collect multiple samples before locking to the 1 kHz sync signal.
            //   Included in the algorithm is an attempt to create a PLL (not
            //   fully implemented at this point)
            if( syncCounter <= PeriodsToAverage ) {
                timerCount = TIM5->CNT;

                if( syncCounter ) { // non-ZERO
                    sumDeltaCount = sumDeltaCount + (int32_t)( timerCount - timerCountPrev );
                } else { // ZERO
                    // Get the timer counter value and reset the count summation value
                    sumDeltaCount = 0;
                }
                timerCountPrev = timerCount;
                syncCounter++;
            } else {   // Obtained 64 samples.  Now average the data.
                initSync    = FALSE;

                // Determine the number of ticks based on the system type
                switch( timer.odr ) {
                    case 100:
                        ticksRequiredForOutput = TICKS_FOR_100_HZ;
                        break;
                    case 200:
                        ticksRequiredForOutput = TICKS_FOR_200_HZ;
                        break;
                    default :
                        // Enter a large number to make the system run slowly so we have to debug
                        ticksRequiredForOutput = 255;
                }

                // What follows is a poor first attempt at a PLL.  Remove and
                //   replace with something that works IF we hope to sync to
                //   other than 1kHz signals.

                /// reload value to correspond to the average value between sync signals
                arrVal = arrPrev + ( sumDeltaCount >> N ); // corrects the signal by half the
                                                           // required amount each time a value
                                                           // is generated.

                /// Limit the value to +/- 15% of nominal (120k counts)
                if( arrVal > 144000 ) {
                    arrVal = 144000;
                } else if( arrVal < 96000 ) {
                    arrVal = 96000;
                }
                TIM5->ARR   = arrVal;
                arrPrev     = arrVal;
                syncCounter = 0;

                // An external sync was detected.  Disable timer 5 and use the external signal as
                //   the data-acquisition timer.
                TIM_Cmd( TIM5, DISABLE );

                //
                gUserSpi.dataRateCounter = 0;
                initSpiCntr = 1;
                timer.oneHundredHertzFlag = 0;
            }
        } else {
            // Increment the counter.  Once the desired limit is reached then
            //   sync the read to the edge and reset the counter.
            cycleCounter++;

            /// 'down-sample' from a 1kHz input signal. For a 500 Hz output, set
            ///   the limit to 2; for a 200 Hz output, set it to 5, etc.  Value
            //    is set above (based on system type).
            if ( cycleCounter >= ticksRequiredForOutput) {
                cycleCounter = 0;

// FIXME: JSM (need to do something similar for UART?)
                // Maybe this command (resetting the timer) can be used to sync
                //   the UART output to the sync line
                //TIM5->EGR = TIM5->EGR | 0x0001;

                if( initSpiCntr ) {
                    gUserSpi.dataRateCounter = gUserSpi.outputDataRate;
                    initSpiCntr = 0;
                }

                /// The sync signal was detected, disable Timer 5 - use the
                ///   external signal as the data-acquisition driver (Note:
                ///   Rate-sensor read moved to accelerometer interrupt)
                OSDisableHook(); ///< Disable user interrupts
                OSSignalBinSem(BINSEM_DATA_ACQ_TIMER);
                OSEnableHook();  ///< Re-enable user interrupts
            }
        }
    }
    EXTI->PR = ONE_PPS_EXTI_LINE;  ///< Clear the interrupt bit
}

/** ***************************************************************************
 * @name InitDataAcquisitionTimer() Set up and initialize the timer interrupt
 *       that drives data acquisition.
 * @brief After timeout, TIM5_IRQHandler, resets the rate-sensor data buffers.
 *        The individual sensor interrupts populates the data buffers when the
 *        sensors indicate 'data-ready'.
 *
 *        Using 32 bit TIM5. Get a sync signal from GPS or the user on TIM2.
 *        use that to trim the TIM5 reload value to have a timer that is the
 *        same rate as the desired outputDataRate
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void InitDataAcquisitionTimer(uint32_t outputDataRate)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;
    double period;

#ifdef SAEJ1939
    if ((gConfiguration.CanOdr == MEMSIC_SAE_J1939_PACKET_RATE_2) ||
        (gConfiguration.CanOdr == MEMSIC_SAE_J1939_PACKET_RATE_5) ||
        (gConfiguration.CanOdr == MEMSIC_SAE_J1939_PACKET_RATE_10) ||
        (gConfiguration.CanOdr == MEMSIC_SAE_J1939_PACKET_RATE_20) ||
        (gConfiguration.CanOdr == MEMSIC_SAE_J1939_PACKET_RATE_25) ||
        (gConfiguration.CanOdr == MEMSIC_SAE_J1939_PACKET_RATE_50) ||
        (gConfiguration.CanOdr == MEMSIC_SAE_J1939_PACKET_RATE_100))
        gEcuConfig.packet_rate = gConfiguration.CanOdr;
    else
        gEcuConfig.packet_rate = MEMSIC_SAE_J1939_PACKET_RATE_100;
    
    can_bus_heart_beat = outputDataRate / gEcuConfig.packet_rate;
#else
    can_bus_heart_beat = outputDataRate / 100;
#endif
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );

    /// Set the timer interrupt period (counter value at interrupt).  Alter this
    /// to sync with a 1PPS or 1kHz signal. ==> SystemCoreClock = 120MHz
    period = (double)( SystemCoreClock ); // >> 1; ///< period = 120 MHz / 2 = 60 MHz
    period = 0.5 * period / (double)outputDataRate;    ///< = period / ODR ==> 60 MHz / 500 Hz = 120,000

    /// Time base configuration
    TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
    TIM_TimeBaseStructure.TIM_Period      = (uint32_t)(period+0.5);
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM5, &TIM_TimeBaseStructure );
    TIM_ARRPreloadConfig( TIM5, ENABLE );

    /// Enable the TIM5 global interrupt, TIM5_IRQn
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    TIM_ITConfig( TIM5, TIM_IT_Update, ENABLE ); ///< TIM Interrupts enable
    /// Ensure an UEV generates an interrupt by deselecting the URS bit in TIM5_CR1
    TIM5->CR1 &= (uint16_t)~TIM_CR1_URS;
    TIM_Cmd( TIM5, ENABLE ); ///< TIM enable counter
}


/** ***************************************************************************
 * @name TIM5_IRQHandler()
 * @brief The value of the timer that triggers the interrupt (period) is based
 *        on the input to the function 'InitDataAcquisitionTimer'. Upon TIM5
 *        interrupt, the rate sensor data buffers are reset when the function
 *        GyroStartReading() is called. The sensor data-ready interrupt handles
 *        the data transfer to the data buffers.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static uint16_t N_per = 0;

// Leave this in here for now so the change can be easily rolled back if
//   something going forward doesn't work quite right.
#define  TIM5_800Hz  1
static uint8_t TIM5_Cntr = 0;
static uint8_t TIM5_CntrLimit = 3;  // 200 Hz Sampling
void TIM5_IRQHandler(void)
{
    static uint8_t initFlag = 1;
    
    tim5_heart_beat++;
    
    if (!(tim5_heart_beat % can_bus_heart_beat))
        OSSignalBinSem(BINSEM_CAN_DATA);
    
    if( initFlag == 1 ) {
        initFlag = 0;

        // For an unaided solution, run taskDAQ at 200 Hz.  An aided solution can only
        //   run at 100 Hz (for now).
        if( UcbGetSysType() <= UNAIDED_AHRS_SYS ) {
            // 200 Hz Sampling
            TIM5_CntrLimit = 3;
        } else {
            // 100 Hz Sampling
            TIM5_CntrLimit = 7;
        }
    }

    OSDisableHook(); ///< Disable user interrupts
#if TIM5_800Hz
    N_per = N_per + 1;
    if( N_per >= 800 ) {
        N_per = 0;
    }

    // New sampling method: 800 Hz Accel/800 Hz Rate-Sensor
    TIM5_Cntr++;
    if( TIM5_Cntr > TIM5_CntrLimit ) {
        TIM5_Cntr = 0;

        // Upon TIM5 timeout, signal taskDataAcquisition() to continue
        OSSignalBinSem(BINSEM_DATA_ACQ_TIMER);
        
        GPIOB->BSRRH = GPIO_Pin_12;
    } else {
        GPIOB->BSRRL = GPIO_Pin_12;
    }
#else
    // Upon TIM5 timeout, signal taskDataAcquisition() to continue
    OSSignalBinSem(BINSEM_DATA_ACQ_TIMER);
#endif

    // reset the interrupt flag
    TIM5->SR = (uint16_t)~TIM_IT_Update;

    OSEnableHook();  ///< Re-enable user interrupts
}


/** ***************************************************************************
 * @name DataAquisitionStart() API
 * @brief used in commands.c to test sensors
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void DataAquisitionStart(void)
{
    // Configure and enable TIM5
#if TIM5_800Hz
    InitDataAcquisitionTimer( TIM5_OUTPUT_DATA_RATE );
#else
    InitDataAcquisitionTimer( timer.odr );
#endif

     /// Enable external sync 1 PPS A0 interrupt
    _InitExternalSync( ENABLE );

    /// Takes longer than the accelerometer and rate sensor readings
    MagnetometerStartReading();
    /// Start acceleration readings
    AccelerometerStartReading();  ///< clear buffer and enable interrupt
}


void _TaskDataAcquisition_Init(void)
{
    _InitSensors();      ///< LOCAL and check for proper start up
    InitializeSensorScaling();  // Set default limits and scale-factors (not really used with UART comm)

    FilterInit();        ///< filter.c load the coefficients for the DSP filters
    CalibrateInit();     /// calibrate.c  Copy cal data into the table

    BITInit(); // BIT.c

    //
    gSpiUcbPacket.systemType = UcbGetSysType(); // set up the system type for SPI MISO xmit
    gSpiUcbPacket.spiAddress = SPI_REG_S1_BURST_READ;

    // FIXME: Place a call to the EEPROM unlock function to unlock the system
    //        so the calibration data can be loaded to the EEPROM.
    //        This should not be unlocked in the final version!
    gAlgorithm.bitStatus.hwStatus.bit.unlockedEEPROM = TRUE;

    /// reset upon entry into main-loop and while doing EEPROM reads/writes.
    InitTimer_Watchdog( DISABLE ); // FIXME turned off for now

    // Initialize the timing variables and set the odr based on the system type
    Initialize_Timing();

    // Initialize the algorithm state
    InitializeAlgorithmStruct(&gAlgorithm);

    // set the scale factors
    InitMagAlignParams();

    DataAquisitionStart(); // local
}


// For use with the 'averaging filter'.  Create a temporary array to hold past
//   values to average data.
static double tmp[6][2] = { {0.0, 0.0},
                            {0.0, 0.0},
                            {0.0, 0.0},
                            {0.0, 0.0},
                            {0.0, 0.0},
                            {0.0, 0.0} };

/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART or SPI.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskDataAcquisition(void)
{
 //   uint8_t functionStatus;

    // This routine sets up the timer. Can use the structure below this point.
    _TaskDataAcquisition_Init();

    // scale ratio is set to one and the sf matrix is identity

    //BITSetRunFlags(); // un comment to run built in test at startup.

    /// Data is provided by the primary sensors (accelerometer and rate-sensor)
    ///   as fast as possible.  Data is obtained from secondary sensors at a
    ///   lower rate.  The rate-sensor data read is synced to TIM5 or an
    ///   external signal.  When the read is commanded and in the data buffer,
    ///   The data-event flag is set and the wait is bypassed.  Data is obtained
    ///   from the buffer and calibrated and filtered.  If not an IMU, the EKF
    ///   is called and an estimate of the system states are made.  Data is
    ///   provided to the user.
    while( 1 )
    {
        TimingVars_Increment();  // Increment once TMR5 triggers

        // state machine to configure, restore and analyze the Self Test results
        BITStartStop();
        PetWatchdog(); // kick the watchdog timer

        // Upon timeout of TIM5 (or user sync), let the process continue
        OS_WaitBinSem(BINSEM_DATA_ACQ_TIMER, OS_TICKS_PER_SECOND);

        // Set the counter to a value that corresponds to seconds after TIM5 is
        //   called and just after the sensors are read.
        gAlgorithm.counter = (uint16_t)( 1.25e-3 * ( ( N_per + (uint16_t)(1.334489891239070E-05 * TIM5->CNT) ) << 16 ) );
        // Increment the timer output value (FIXME: is this in the right spot?
        //   Should it be in the algorithm if-statement below?)
        gAlgorithm.timer = gAlgorithm.timer + gAlgorithm.dITOW;

        // Disable the accelerometer interrupt to prevent additional data reads
        //   while processing current data
        //AccelDataReadyInt(DISABLE);

        // 
        if( getUserCommunicationType() == SPI_COMM )
        {
            /// Reset (deselect) the DR pin. The accelerometer and rate-sensor
            ///  data are available - processing has begun old SPI data is no
            ///  longer valid. User should not poll for data until it has
            ///  finished processing at the end of this task.
            if( gUserSpi.EnableDataReady )
            {
                if( gUserSpi.DataReadyPolarity ) {
                    _DataReadyPin_SetLow();
                } else {
                    _DataReadyPin_SetHigh();
                }
            }
        }

        // After HW bias applied - when complete Bias is removed
        // has run flag inside, void so no context switch
        BITCollectData( reading );

        /// ********** Get the accelerometer and rate sensor data  **********
        ///   save the data in the 'reading' array
        AccelerometerGetLastReading( &reading[ACCEL_START] );
        GyroGetLastReading( &reading[RATE_START] );
        GyroTempGetLastReading( &reading[GYRO_TEMP] );
        _Temp_MovingAverage( &reading[0]);

        // Re-enable the accelerometer interrupt
        //AccelDataReadyInt(ENABLE);

        /// When the daq process is running and the accel and rate sensor data
        ///   is valid, get the magnetometer data through interrupt isr
        if( timer.oneHundredHertzFlag ) {   // <-- This flag changes at 200 Hz
            if( OSReadEFlag( EFLAGS_DATA_READY ) & EF_DATA_MAG_READY )
            {
                if( gCalibration.productConfiguration.bit.hasMags ) {
                    MagnetometerGetLastReading( &reading[MAG_START] );
                    MagnetometerStartReading();
                    OSClrEFlag( EFLAGS_DATA_READY, EF_DATA_MAG_READY );
                } else {
                    reading[XMAG] = 0x00;
                    reading[YMAG] = 0x00;
                    reading[ZMAG] = 0x00;
                }
            }
        }

//        // External magnetometer values selected, overwrite the magnetometer
//        //   values stored in gAlgorithm.scaledSensors and
//        //   gAlgorithm.scaledSensors_q27 (combine with the above code)
//        if( ( gCalibration.productConfiguration.bit.hasMags == FALSE ) &&
//            ( gConfiguration.userBehavior.bit.useMags == TRUE ) )
//        {
//            // Overwrite the magnetometer values
//            gAlgorithm.scaledSensors[XMAG] = 0.0;
//            gAlgorithm.scaledSensors[YMAG] = 0.0;
//            gAlgorithm.scaledSensors[ZMAG] = 0.0;
//
//            // If needed, compute the Q27 values for the magnetometers
//            gAlgorithm.scaledSensors_q27[XMAG] = (int32_t)(gAlgorithm.scaledSensors[XMAG] * 134217728);
//            gAlgorithm.scaledSensors_q27[YMAG] = (int32_t)(gAlgorithm.scaledSensors[YMAG] * 134217728);
//            gAlgorithm.scaledSensors_q27[ZMAG] = (int32_t)(gAlgorithm.scaledSensors[ZMAG] * 134217728);
//        }

    //if (!isCalibrateTableValid()) {
        ekfCounter++;
        /// sensor reading => ranges that Nav-View and the calibration routine
        ///   use; data available to the rest of the program (gAlgorithm.rawSensors)
        _ConvertToXBowScaling( &reading[0] );
//FIXME: may have inadvertantly changed this during an update.  What should it be?

        // The values used by the filter (gAlgorithm.rawSensors) are generated
        //   in _ConvertToXBowScaling (filtering should be done after
        //   _ConvertToXBowScaling)
        // Disable filtering except for the magnetometers
        CalibrateFilter();

        /// Apply calibration data to the raw sensor data. Return
        ///   (float) results in gAlgorithm.scaledSensors.
        CalibrateApply();

// ------------------------------ Algorithms ------------------------------
//FIXME: may have inadvertantly changed this during an update.  What should it be?
        handleBITandStatus();
        handleOverRange(); // bit over range

        // Average 200 Hz data
        //   FIXME: JSM (this should operate on the counts instead)
        if( timer.odr == ODR_200_HZ ) {
            for( uint8_t elemNum = XACCEL; elemNum <= ZRATE; elemNum++ ) {
                tmp[elemNum][0] = 0.5 * ( tmp[elemNum][1] + gAlgorithm.scaledSensors[elemNum] );
                tmp[elemNum][1] = gAlgorithm.scaledSensors[elemNum];
                gAlgorithm.scaledSensors[elemNum] = tmp[elemNum][0];
            }
        }

        // If an IMU, do not call the EKF algorithms (delays the write of the
        //   sensor values to the output)
    if( gSpiUcbPacket.systemType > IMU_9DOF_SYS ) {  // or if( gCalibration.productConfiguration.bit.algorithm == TRUE ) {
#ifdef RUN_PROFILING
            uint32_t ekfStartTime = TimeNow();
#endif

            // Call the EKF at 100 or 200 Hz (at 200 Hz, run the EKF upon each
            //   pass through the loop.  At 100 Hz, run it during every other
            //   pass.)
            if( gAlgorithm.callingFreq == ODR_200_HZ ) {
                EKF_Algorithm();
            } else {
                if(timer.oneHundredHertzFlag==1) {
                    EKF_Algorithm();
                }
            }

#ifdef RUN_PROFILING
            gEkfElapsedTime = TimeNow() - ekfStartTime;
            gEkfAvgTime = 0.9 * gEkfAvgTime + 0.1 * gEkfElapsedTime;
            if (gEkfElapsedTime > gEkfMaxTime) {
                gEkfMaxTime = gEkfElapsedTime;
            }
#endif

            // Mag Cal (follows Kalman filter as the innovation routine
            //   calculates the euler angles and the magnetic vector in the
            //   NED-frame)
            if( gCalibration.productConfiguration.bit.hasMags ) {
                /*functionStatus = */MagAlign(); // only does this on align
            }
        }
        //} // calibrate table check
//} // do not call algorithm bit set 

// ------------------------------ SPI interface ------------------------------
        /// Set data ready flag high. (Reset)
        ///   SPI - when a read of the data register is made
        ///   UART - N/A: data passed out by the UART code
        /// Populate SPI data register (with calibrated data).
        ///   Dependent upon the ODR of 400 Hz
        if( getUserCommunicationType() == SPI_COMM )
        {
            /// Populate the SPI data register
            if( gUserSpi.outputDataRate != 0 )
            {
                /// Decimate the data from 200 Hz based on packet rate divider
                uint16_t counterVal = gUserSpi.dataRateCounter;
                if( counterVal >= gUserSpi.outputDataRate )
                {
                    // Reset the counter
                    gUserSpi.dataRateCounter = 0;

                    FillSpiDataRegister_Sensors(); ///< at the prescribed rate
                    //FillSpiDataRegister_Algorithms();
                    FillSpiDataRegister_BIT();

                    /// with sensor data. 16-bit shorts assembled from
                    ///   2 bytes - status, rates, accels, temp - in
                    ///   gUserSpi.DataRegister used by the DMA code.
 // FIXME: need to detect a JD
                    FillSpiBurstRegister_Sensors(); // JD
                    LoadUcbSPIBuffer(&gSpiUcbPacket); // not JD

                    /// Toggle data-ready line - ready for output to user.
                    if( gUserSpi.EnableDataReady ) {
                        if( gUserSpi.DataReadyPolarity ) {
                            _DataReadyPin_SetHigh();
                        } else {
                            _DataReadyPin_SetLow();
                        }
                    }
                }

                /// Increment when value equal to outputDataRate.
                if( timer.odr == ODR_100_HZ ) {
                    // When not an IMU, TaskDAQ operates at 100 Hz
                    gUserSpi.dataRateCounter = gUserSpi.dataRateCounter + 2;
                } else {
                    // IMU
                    gUserSpi.dataRateCounter++;
                }
            }
        } else {
            // Set the semaphore to let the system know when to output UART
            //   (NavView) data.  Additionally, this prevents the 1.2 msec delay
            //   that is seen on the UART data by the OS pre-empting the UART
            //   output in taskUserCommunication.c.  However, can cause a
            //   problem with the EEPROM write (FIXME: JSM).
            OSSignalBinSem( BINSEM_NAVVIEW_DATA_READY );
        }
    }
}


/** ***************************************************************************
 * @name _InitSensors()
 * @brief used in the 380. Check for errors and return the status of the sensors.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void _InitSensors( void )
{
    uint32_t range;
    uint32_t outputDataRate;

    /// ========== Accelerometers ==========
    range = (uint32_t)gCalibration.AccelSensorRange;
    // Select the ODR of the accelerometer
    if( UcbGetSysType() <= UNAIDED_AHRS_SYS ) {
        // Unaided: for an imu, set the ODR to 800 Hz
        outputDataRate = (uint32_t)800;
    } else {
        // Aided-AHRS/VG or INS: set the ODR to 400 Hz
        outputDataRate = (uint32_t)400;
    }

    if( !InitAccelerometer(FALSE) ) {
        ERROR_STRING("Failed to init accel.\r\n");
        gAlgorithm.bitStatus.hwBIT.bit.sensorError        = 1;
        gAlgorithm.bitStatus.hwBIT.bit.AccelerometerError = 1;
    }
    AccelerometerConfig(&range, &outputDataRate);

    /// ========== Rate sensors ==========
    /// Note: the rate sensor dictates the data output rate since the
    ///       accelerometer outputs data at a prdefined rate of 800 Hz (due to
    ///       alignment of data samples).
    range          = (uint32_t)gCalibration.GyroSensorRange;
#ifdef GYRO_BMI160
    outputDataRate = (uint32_t)1600;
#else
    outputDataRate = (uint32_t)RATE_SENSOR_ODR;
#endif
    if( !InitGyro() ) {
        ERROR_STRING("Failed to init gyro.\r\n");
        gAlgorithm.bitStatus.hwBIT.bit.sensorError    = 1;
        gAlgorithm.bitStatus.hwBIT.bit.RateGyroError  = 1;
    }
    GyroConfig(&range, &outputDataRate); // default 8.1 Gauss
    /// ----- Magnetometers -----
    range = (uint32_t) gCalibration.MagSensorRange;

    if( !InitMagnetometer() ) {
        ERROR_STRING("Failed to init mag.\r\n");
        gAlgorithm.bitStatus.hwBIT.bit.sensorError       = 1;
        gAlgorithm.bitStatus.hwBIT.bit.MagnetometerError = 1;
    }
    MagnetometerConfig(&range);
}

/** ***************************************************************************
 * @name _ConvertToXBowScaling() LOCAL
 * @brief XBOW packet scaling. Output data is placed in 'gAlgorithm.rawSensors'
 *
 * Adjust the values in gAlgorithm.rawSensors to work with the scaling that
 *  Nav-View uses to display counts and convert into (virtual) voltages (0-5V)
 *   -------- Rate sensor, accelerometer, and magnetometer readings --------
 *
 * NOTE: To make the factory packets work with Nav-View, it is necessary to
 *       shift the word(data contained in rawSensors) to the right by 9 bits.
 *       This limits the maximum value in 'rawSensors' to 2^23 (the scaling
 *       factor in Nav-View)
 * @param [in] reading - input raw data
 * @retval N/A
 ******************************************************************************/
void _ConvertToXBowScaling( int16_t* reading )
{
    uint32_t boardTemp  = 0;
    uint32_t accelTemp  = 0;
    uint32_t gyroTemp   = 0;

    /// Convert from int16 to uint32 and shift results to uint23_t (for proper
    ///   sensor-data scaling by Nav-View)
    gAlgorithm.rawSensors[ XACCEL ] = CONVERT_380_TO_XBOW( reading[ XACCEL ] ) >> 9;
    gAlgorithm.rawSensors[ YACCEL ] = CONVERT_380_TO_XBOW( reading[ YACCEL ] ) >> 9;
    gAlgorithm.rawSensors[ ZACCEL ] = CONVERT_380_TO_XBOW( reading[ ZACCEL ] ) >> 9;
    gAlgorithm.rawSensors[ XRATE ]  = CONVERT_380_TO_XBOW( reading[ XRATE ] )  >> 9;
    gAlgorithm.rawSensors[ YRATE ]  = CONVERT_380_TO_XBOW( reading[ YRATE ] )  >> 9;
    gAlgorithm.rawSensors[ ZRATE ]  = CONVERT_380_TO_XBOW( reading[ ZRATE ] )  >> 9;
    gAlgorithm.rawSensors[ XMAG ]   = CONVERT_380_TO_XBOW( reading[ XMAG ] )   >> 9;
    gAlgorithm.rawSensors[ YMAG ]   = CONVERT_380_TO_XBOW( reading[ YMAG ] )   >> 9;
    gAlgorithm.rawSensors[ ZMAG ]   = CONVERT_380_TO_XBOW( reading[ ZMAG ] )   >> 9;

    /// -------- Board and accelerometer temperature readings --------
    /// Board temperature scaling factor (Nav-View) = 2^16. << 16 data short
    ///   to remove the scaling applied above.  NOTE: Shifting by
    ///   17 bits seems to place it in a more realistic voltage range (FIXME: verify this)
    boardTemp = CONVERT_380_TO_XBOW( reading[ TEMP_SENSOR ] );
    gAlgorithm.rawSensors[ BTEMP ]  = boardTemp >> 16;

    /// The rate sensor and accelerometer temperatures are scaled by 2^30.
    /// Nav-View may not recognize all three axis.

    /// NOTE: all temperatures for board and sensors are gyro temp for
    ///       compatibility with Nav View and the Calibration Applications
    /// ----- Accelerometer temperature -----
    accelTemp =  boardTemp >> 2;
    gAlgorithm.rawSensors[ XATEMP ] = accelTemp;
    gAlgorithm.rawSensors[ YATEMP ] = accelTemp;
    gAlgorithm.rawSensors[ ZATEMP ] = accelTemp;

    /// ----- Rate sensor temperature -----
    /// 30-bit unsigned-integer
    gyroTemp = CONVERT_380_TO_XBOW( reading[ GYRO_TEMP ] ) >> 2;
    gAlgorithm.rawSensors[ XRTEMP ] = gyroTemp;
    gAlgorithm.rawSensors[ YRTEMP ] = gyroTemp;
    gAlgorithm.rawSensors[ ZRTEMP ] = gyroTemp;
}


/** ***************************************************************************
 * @name LimitInt16Value() API utility function to saturate an input value
 * @brief saturate
 * @param [in] value - input value
 * @param [in] limit - limit value
 * @retval limited input value
 ******************************************************************************/
int16_t LimitInt16Value( int16_t value,
                         int16_t limit )
{
    if( value > limit ) {
        return limit;
    } else if( value < -limit ) {
        return -limit;
    } else {
        return value;
    }
}


// Change the following two values together so they are consistent
//   (TEMP_AVG_PERIOD = 2^TEMP_BIT_SHIFT).  The averaging period is
//   in seconds.
#define  TEMP_AVG_PERIOD  8
#define  TEMP_BIT_SHIFT   3

/*
*/
uint8_t _Temp_MovingAverage(int16_t *reading)
{
    static uint8_t initTempBuff = 1;
    static int16_t tempBuff[TEMP_AVG_PERIOD];
    static uint8_t tempBuffInd = 0;
    static int32_t tempBuffTotal = 0;

    // Get temperature at 1 Hz - MAX21000 Datasheet.  Average the last eight
    //   data points.
    if( initTempBuff <= TEMP_AVG_PERIOD ) {
      // Startup: buffer not filled.  Populate it so the moving average works properly.
        initTempBuff = initTempBuff + 1;

        tempBuffTotal = tempBuffTotal + reading[GYRO_TEMP];
        tempBuff[initTempBuff-2] = reading[GYRO_TEMP];
        reading[TEMP_SENSOR]     = reading[GYRO_TEMP];
        tempBuffInd = 0;
    } else {
        // Initialization complete.  Read and average value at a 1 Hz rate.
        if( timer.basicFrameCounter == 0 ) {
            // Subtract off the oldest value and add in the latest reading
            tempBuffTotal = tempBuffTotal - tempBuff[tempBuffInd] + reading[GYRO_TEMP];
            // Replace the oldest item in the buffer with the newest
            tempBuff[tempBuffInd] = reading[GYRO_TEMP];
            // Increment index and reset once it reaches eight
            tempBuffInd++;
            if( tempBuffInd >= TEMP_AVG_PERIOD ) {
                tempBuffInd = 0;
            }

            // Compute the moving-average
            reading[TEMP_SENSOR] = (int16_t)( tempBuffTotal >> TEMP_BIT_SHIFT );
        }
    }

    return 1;
}

/** ***************************************************************************
 * @name DataAquisitionStop() API
 * @brief Called from user command interface. Once running, the data acquisition
 *   process should not stop (unless commanded).
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void DataAquisitionStop(void)
{
    TIM_Cmd(TIM5, DISABLE);
}

