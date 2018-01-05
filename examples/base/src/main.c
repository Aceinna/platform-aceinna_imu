/** ***************************************************************************
 * @file   main.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  main is the center of the system universe, it all starts here. And never ends.
 * entry point for system (pins, clocks, interrupts), data and task initalization.
 * contains the main processing loop. - this is a standard implementation
 * which has mainly os functionality in the main loop
 ******************************************************************************/
#include <stddef.h>

#include "stm32f2xx.h"
#include "bsp.h"

//#define LOGGING_LEVEL LEVEL_STREAM
#include "debug.h" // boot cause message

#include "stm32f2xx_conf.h"
#include "salvodefs.h"
#include "commandLine.h"
#include "debug_usart.h"
#include "taskDataAcquisition.h"
#include "taskUserCommunication.h"
#include "UserCommunication_SPI.h"
// for spi selection (configuration bit)
#include "dmu.h"  // BOOL
#include "configureGPIO.h"
#include "gps.h"
#include "can.h"

#include "xbowsp_init.h"
#include "ucb_packet.h"

#include "WorldMagneticModel.h"

void _TaskCommandLine(void);

void ControlPortInit(void)
{
   	GPIO_InitTypeDef GPIO_InitStructure;
    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // PD2 for 120Ohm switch control
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/** ***************************************************************************
 * @name main() The DMU380 firmware initialization and entry point
 * @brief creates tasks for data collection SPI1 and user communications SPI3
 *        or USART serial stream (Nav-View) debug serial console messaging
 *
 * @param [in] N/A
 * @param [out] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/
int main(void)
{
    // Initialize the system clock, PLL, etc
    SystemInit();            // system_stm32f2xx.c
    //SetIntVectorOffset(APP_NVIC_OFFSET);

    SystemCoreClockUpdate(); // system_stm32f2xx.c

    // In the case of the HSI clock configuration, change "system_stm32fxx.c"
    //   based on the Excel-generated file and comment out RCC_config() in
    //   BSP_init.  The variable 'tmp' is set based on whether the internal or
    //   external clock is used by the processor.  Both will generate
    //   approximately a 120 MHz system clock using PLLs.
    BSP_init();              // bsp.c

    // Read from EEPROM and write the configuration and calibration data into RAM
    initConfigureUnit(); // xbowsp_init.c, s_eeprom.c outputs the CRC

    ControlPortInit();
    // Determine which algorithm to run based on the product-type and device
    //   settings (specifically has/useGPS)
    if( UcbGetSysType() > IMU_9DOF_SYS ) {
        if( IsInternalGPS() ) {
            // Device has an internal GPS (can run the INS algorithm)
            setAlgoType(INS_ALGO);
        } else {
            // Device does not have an internal GPS (can run INS algorithm if
            //   GPS data is provided by an external source)
            if ( UseExtUSARTGps() == true) { // useGPS
                setAlgoType(INS_ALGO);  //  hasGPS = false, useGPS = true
            } else {
                setAlgoType(AHRS_ALGO); //  hasGPS = false, useGPS = false
            }
        }
    } else {
        // IMU (no algorithm)
        setAlgoType(NO_ALGO);
    }

    // Initialize the DEBUG USART (serial) port
    // normal debug baud rate - Disable the debug derial communication for release builds
    InitDebugSerialCommunication( 38400); // debug_usart.c
    DEBUG_STRING("\r\nDMU380 System\r\n");

    // Add a delay to allow the system to stabilize after the reset line (nRst)
    //   is released
    DelayMs(300);

    // --------------------------- BOOT CAUSE MESSAGING -----------------------
    if( RCC_GetFlagStatus( RCC_FLAG_PORRST ) ) { // Power-on/power-down reset
        ERROR_STRING("\r\nNormal power-on.\r\n");
    } else if( RCC_GetFlagStatus( RCC_FLAG_BORRST ) ) { // Brown-out reset
        ERROR_STRING("\r\nReset due to power brown-out.\r\n");
    } else if( RCC_GetFlagStatus( RCC_FLAG_IWDGRST ) ) { // Watchdog reset
        ERROR_STRING("\r\nReset due to watch dog timeout.\r\n");
    } else if( RCC_GetFlagStatus( RCC_FLAG_LPWRRST ) ) { // Low power
        ERROR_STRING("\r\nReset due to low-power.\r\n");
    } else if( RCC_GetFlagStatus( RCC_FLAG_SFTRST ) ) {  // Software reset
        ERROR_STRING("\r\nReset due to software reset.\r\n");
    }
    RCC_ClearFlag(); ///< reset flags - stm32f2xx_rccc.c
    // ---------------------------- BOOT CAUSE MESSAGING -----------------------

    OSInit(); ///< data structures, pointers, and counters (prepare for mutitasking)

    /// Initialize data-ready (DR), configuration, and (1-PPS) pins as input pins
    ///   (the board is configured according to the signal levels on these pins)
    InitBoardConfiguration_GPIO();  // configureGPIO.c
    ReadUnitConfiguration_GPIO();   // configureGPIO.c

    // Debugging pin IO3 will be used to indicate to the user when the rate-
    //   sensor is read (with a rising edge).  Start operation by setting this
    //   pin low.
    GPIOB->BSRRH = GPIO_Pin_11;  // Set IO3 low

    /// Create RTOS tasks:
    ///   Col. 1: function name, Col. 2: pointer to the task control block, Col. 3: task priority
    OSCreateTask( TaskUserCommunication, USER_COMMUNICATION_TASK, USER_COMMUNICATION_PRIORITY ); // priority 8
    OSCreateTask( TaskDataAcquisition,   DATA_ACQ_TASK,           DATA_ACQ_PRIORITY           ); // priority 3
    OSCreateTask( _TaskCommandLine,      COMMAND_LINE_TASK,       COMMAND_LINE_PRIORITY       ); // priority 10  (FIXME: JSM - Feb 18)

#ifdef GPS
    // Start GPS task if the system in not using SPI to communicate with the
    //   user AND the device is NEITHER an IMU nor an unaided-AHRS/VG
    if (getUserCommunicationType() == UART_COMM) {
        if ( (UcbGetSysType() > UNAIDED_AHRS_SYS) ) {
            // can't run at the same time as SPI
            OSCreateTask( TaskGps,                GPS_TASK, GPS_PRIORITY ); // priority 14
            OSCreateTask( TaskWorldMagneticModel, WMM_TASK, WMM_PRIORITY ); // priority 15
        }
    }
#endif

#ifdef MEMSIC_CAN
    OSCreateTask( TaskCANCommunication,                CAN_TASK, CAN_PRIORITY ); // priority 9
#endif




    /// Assign an event control block (ecb) to each semaphore.  Signaling a
    ///   waiting task is done via the RTOS function, OSSignalBinSem(); semaphore
    ///   reset upon call of WaitBinSem().
    OSCreateBinSem( BINSEM_SERIAL_RX, 0 );
    OSCreateBinSem( BINSEM_NAVVIEW_DATA_READY, 0 );  // Semaphore to sync Nav-View data out with TIM5
    OSCreateBinSem( BINSEM_CAN_DATA, 0 );

    // Semaphore to tell DAQ process that timer IRQ has triggered
    OSCreateBinSem( BINSEM_DATA_ACQ_TIMER, 0 );

    //OSCreateBinSem( BINSEM_USER_COMMUNICATION, 0 );
//    if (getUserCommunicationType() == UART_COMM) {
//        OSCreateBinSem( WMM_SOLN_AVAILABLE, 0 );
//    }

    /// may want to disable interrupts until here... OSTimer may be called before
    ///   OS running? Create event flags
    OSCreateEFlag( EFLAGS_DATA_READY, EFLAGS_DATA_READY_CB_P, 0x00 );
    OSCreateEFlag( EFLAGS_USER,       EFLAGS_USER_CB_P,       0x00 );

//InitTimer_Watchdog( ENABLE );
    // Run the tasks...
    while (1) {
        OSSched();
    }
}


/** ****************************************************************************
 * @name TaskCommandLine() LOCAL task callback function
 * @brief handles debug serial console command line messaging
 *        When the USART interrupt handler indicates a RX on the USART line,
 *        toggle the LED and parse the received message using the commandTable
 *
 * @param [in] gCommands - command line command and arguments
 * @param [out] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/
void _TaskCommandLine(void)
{
    CmdPrintPrompt(); // out to serial console

    while (1) {
        /// context switch and wait for the semaphore, BINSEM_SERIAL_RX, to
        /// change state: occurs in the USART interrupt handler (DEBUG_USART_IRQ)
        OS_WaitBinSem( BINSEM_SERIAL_RX, OSNO_TIMEOUT );
        CmdLineLookup( gCommands ); // commandLine.c
    }
}

