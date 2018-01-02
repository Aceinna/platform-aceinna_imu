/** ***************************************************************************
 * @file   taskUserCommunication.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * End user communication with the DMU380:
 * - Nav-View (UART) maximum rate of 100 Hz (due to wait in the function below)
 *    handled by the Memsic ucb (Unified Code Base) - handle_packet, send_packet,
 *    extern_port.c, ucb_packet.c comm_buffers.c
 * - SPI communication is an external interrupt driven (asynchronous) bus
 ******************************************************************************/
#include "stm32f2xx_conf.h"
#include "salvodefs.h"
#include "stm32f2xx.h"
#include "dmu.h"

#include "debug_usart.h"
#include "debug.h"

#include "extern_port.h"
#include "ucb_packet.h"
#include "uart.h"
#include "timer.h"
#include "xbowsp_algorithm.h"
#include "xbowsp_generaldrivers.h"
#include "xbowsp_init.h"
#include "taskUserCommunication.h"

// for spi interface
#include "spi.h"
#include "boardDefinition.h"
#include "UserCommunication_SPI.h"
   
// for can interface
#include "can.h"
#include "sae_j1939.h"

#include "configureGPIO.h"
#include "bsp.h" // for definition of LED4

extern uint32_t can_bus_heart_beat;
extern uint32_t tim5_heart_beat;

static void _ProcessUcbCommands(void);
static void _ProcessContUcbPkt(void);

const uint16_t MAIN_LOOP_PERIOD = 10; /// ms

uint8_t divideCount = 1; /// continuous packet rate divider

static UcbPacketStruct continuousUcbPacket; // ucb_packet.h
static UcbPacketStruct primaryUcbPacket;    /// all other data

//
uint32_t userCommunicationType;
inline uint32_t getUserCommunicationType(void)
{
    return userCommunicationType;
}

void setUserCommunicationType(uint32_t commType) {
    userCommunicationType = commType;
}


#include "TimingVars.h"

/** ***************************************************************************
 * @name TaskUserCommunication() user communication task callback
 * @brief Initialize hw selected interface DR floating - SPI3 or DR low -
 *        serial (NAV VIEW)
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void TaskUserCommunication(void)
{
    /// Select UART or SPI (if DR pin is pulled LOW then select UART, else SPI).
    if( GPIO_ReadInputDataBit( DATA_READY_PORT, DATA_READY_PIN ) == GPIO_INPUT ) {
        setUserCommunicationType(UART_COMM);

        /// user UART - initialize including the underlying UCB interface
        ExternPortInit(); // extern_port.c
    } 
    else { /// SPI
        setUserCommunicationType(SPI_COMM);

        InitPin_GPIO( DATA_READY_CLK,
                      DATA_READY_PORT,
                      DATA_READY_PIN,
                      GPIO_OUTPUT ); // DR -> SPI master on ready for MISO "read"

        InitCommunication_UserSPI();
    }
//#else  
//    //setUserCommunicationType(UART_COMM);
//    //ExternPortInit();
//    setUserCommunicationType(CAN_BUS);
//    InitCommunication_UserCAN();
//    if (!(gEcuConfigPtr->ecu_name.words)) {
//      gEcuConfigPtr->ecu_name.bits.function = MEMSIC_SAE_J1939_FUNCTION;
//      gEcuConfigPtr->ecu_name.bits.manufacture_code = MEMSIC_SAE_J1939_MANUFACTURE_CODE;
//      gEcuConfigPtr->ecu_name.bits.identity_number = gCalibration.serialNumber & 0x1fffff;
//    }
//    sae_j1939_initialize();
//#endif

    
    // Main loop for the task
    while( 1 )
    {   /// data-ready [DR] pin left floating - SPI
        if( userCommunicationType == SPI_COMM )
        {
            /// User SPI is interrupt driven when the RX buffer is filled.
            ///   Incorporate a one second delay for the communication task.
            ///   Context-switch upon time-out.
            OS_Delay( OS_TICKS_PER_SECOND );

       }
//#ifdef MEMSIC_CAN
//        else if( userCommunicationType == CAN_BUS ) {
//          OS_WaitBinSem(BINSEM_CAN_DATA, OS_TICKS_PER_SECOND);
//#ifdef MEMSIC_CAN_HOST
//          ecu_host_test(19);
//#else
//          // ecu process
//          ecu_process();
//        }
//#endif // _CAN_HOST
//#endif
        else {   /// SERIAL
            /// Wait here for data-ready semaphore.  Set by TIM5 interrupt at a 200 Hz rate to
            ///   ensure that the OS_Wait does not exhibit a large delay.  A context switch will
            ///   occur upon every call to OS_WaitBinSem.

// Add logic here that checks the packet before deciding which semaphores, etc so Nav-View
//   can sync to the 100 Hz timing var.
//
// To set to 200 Hz output, set this #if to zero and comment out the if-statement using the
//   variable timer.oneHundredHertzFlag
//
#if 1
            // 100 Hz output
            OS_WaitEFlag( EFLAGS_USER,
                          EF_USER_ALL,
                          OSALL_BITS,
                          MAIN_LOOP_PERIOD / SALVO_TIMER_PRESCALE );
//        GPIOB->BSRRH = GPIO_Pin_12;
            _ProcessUcbCommands();
            _ProcessContUcbPkt();
            uart_BIT( USER_COMM_UART ); // kUserA_UART
#else

#if 1
            OS_WaitBinSem( BINSEM_NAVVIEW_DATA_READY, OSNO_TIMEOUT );

            /// Process UART data at a 100 Hz rate despite the task being called at 200 Hz
            if( timer.oneHundredHertzFlag ) {
                _ProcessUcbCommands();
                _ProcessContUcbPkt();
                uart_BIT( USER_COMM_UART ); // kUserA_UART
//        GPIOB->BSRRH = GPIO_Pin_12;
            }
#else
// FIXME: JSM -- Replace the 'notimeout' with the same timeaout as above.  This
//               may solve the EEPROM write problem.
            //BINSEM_NAVVIEW_DATA_READY
            OS_WaitBinSem( BINSEM_NAVVIEW_DATA_READY,
                           MAIN_LOOP_PERIOD / SALVO_TIMER_PRESCALE );
            if( timer.oneHundredHertzFlag ) {
//        GPIOB->BSRRH = GPIO_Pin_12;
                _ProcessUcbCommands();
                _ProcessContUcbPkt();
                uart_BIT( USER_COMM_UART ); // kUserA_UART
            }
#endif

#endif
        }
    }
}

/** ****************************************************************************
 * @name _ProcessUcbCommands
 *
 * @brief  This routine will test for the a port to be assigned to the UCB
 * function and test that port for a received packet.  If the packet is received
 * it will call a handler.
 *
 * Trace:
 * [SDD_PROCESS_USER_PORTS_01 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_USER_PORTS_02 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_USER_PORTS_03 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_COMMANDS_SEQ <-- SRC_PROCESS_UCB_COMMANDS]
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void _ProcessUcbCommands (void)
{
    /// check all ports for received packets and handle appropriately
    if ( HandleUcbRx (PRIMARY_UCB_PORT, &primaryUcbPacket) == TRUE ) {
        HandleUcbPacket( PRIMARY_UCB_PORT,
                         &primaryUcbPacket ); // handle packet.c
    }
} /* end ProcessUcbCommands() */


/** ****************************************************************************
 * @name _ProcessContUcbPkt
 *
 * @brief This generates the automatic transmission of UCB packets. The specified
 * packet type will be sent at some multiple of the 10 mSec acquisition rate.
 * This allows large packets that require more time to be sent.
 *
 * Trace:
 * [SDD_PROCESS_PRIMARY_01 <-- SRC_PROCESS_PRIMARY]
 * [SDD_PROCESS_PRIMARY_02 <-- SRC_PROCESS_PRIMARY]
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void _ProcessContUcbPkt (void)
{
    uint8_t type [UCB_PACKET_TYPE_LENGTH];

    if (gConfiguration.packetRateDivider != 0) { ///< check for quiet mode
        if (divideCount == 1) {
            /// get enum for requested continuous packet type
            type[0] = (uint8_t)((gConfiguration.packetType >> 8) & 0xff);
            type[1] = (uint8_t) (gConfiguration.packetType & 0xff);

            /// set continuous output packet type based on configuration
            continuousUcbPacket.packetType = UcbPacketBytesToPacketType(type);
            SendUcbPacket(PRIMARY_UCB_PORT,
			              &continuousUcbPacket);
            divideCount = (uint8_t)(gConfiguration.packetRateDivider);
        } else {
            --divideCount;
        }
    }
} /* end ProcessContUcbPkt() */

