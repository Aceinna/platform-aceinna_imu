/** ***************************************************************************
 * @file   i2c.c I2C driver functions including both interrupt and DMA handlers
 * @author jsun
 * @date    2011-02-18 16:56:57 -0800 (Fri, 18 Feb 2011)
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details I2C1 Magnetomter, Temperature
 *          I2C3 Accelerometer
 * 12C specific routines to deal with bus communications
 *
 * $Revision: 15960 $
 *****************************************************************************/
#include <stdint.h>

#include "stm32f2xx_conf.h"
#include "stm32f2xx.h"
#include "salvo.h"
#include "i2c.h"
#include "boardDefinition.h"
#include "debug.h"
#include "dmu.h"
#include "xbowsp_algorithm.h"


#define kI2C1   0
#define kI2C3   1
#define kI2C2   2
#define NUM_I2C 3

//
// Set the following to 1 to use the ST function calls
#define EXTRACT_I2C_LOGIC_NOT 1

#define I2C_TIMEOUT   2000
#define ERROR_TIMEOUT 0x100

enum eI2CState{
    i2cStateAddrSend,            ///< Start already sent, now send address.
    i2cStateAddrWaitAck,         ///< Wait for ACK/NACK on address sent.
    i2cStateReStart,             ///< Repeat start.
    i2cStateDataSend,            ///< Send data.
    i2cStateDataWaitAck,         ///< Wait for ACK/NACK on data sent.
    i2cStateRxStart,             ///< Wait for data.
    i2cStateRxData,              ///< Wait for data.
    i2cStateRxDataDMA,           ///< Wait for data to finish through DMA.
    i2cStateRxWaitStop,          ///< Wait for STOP to have been transmitted.
    i2cStateError,
    i2cStateDone                 ///< Transfer completed successfully.
} ;

static  struct {
    I2C_TypeDef        *I2Cx;
    uint32_t           dmaFlags;
    DMA_Stream_TypeDef *dmaStream;

    enum eI2CState     state;
    uint32_t           lastEvent;
    uint32_t           thisEvent;
    uint32_t           timeout;

    volatile uint32_t error;
    uint32_t          errorCount;
    uint8_t           notInUse:1;

    uint8_t           slaveAddr;
    uint8_t           txSize;
    uint8_t           rxSize;

    uint8_t           txDataMem[2]; ///< two bytes of memory to hold things

    uint8_t           *txData;
    uint8_t           *rxData;

    void (*callback)(void);
}  gI2C[NUM_I2C];

static void _i2c_event(int i); // delayed service handler

/** ***************************************************************************
 * @name _i2cx_to_index()  LOCAL
 * @brief I2C1 - magHMC5883L 400khz
 *        I2C3 - accelIMMA8451Q
 * @param [in] I2Cx - base address
 * @retval index into gI2C global
 ******************************************************************************/
static int _i2cx_to_index(I2C_TypeDef *I2Cx)
{

    if (I2C1 == I2Cx)  {
        return kI2C1;
    }
    if (I2C3 == I2Cx) {
        return kI2C3;
    }

    if (I2C2 == I2Cx) {
        return kI2C2;
    }

    return 0;
}

/** ***************************************************************************
 * @name _i2c_clear_arbitration_lost()  LOCAL reset the pins
 * @brief
 *
 * @param [in] i - index into gI2C global
 * @retval N/A
 ******************************************************************************/
static void _i2c_clear_arbitration_lost(int i)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_TypeDef      *I2Cx = gI2C[i].I2Cx;
    GPIO_TypeDef     *port;
    int              scl;
    int              sda;

   if (I2C1 == I2Cx)  {
        sda  = I2C1_SDA_PIN;
        scl  = I2C1_SCL_PIN;
        port = I2C1_SCL_GPIO_PORT;
    } else if (I2C3 == I2Cx) {
        sda  = I2C3_SDA_PIN;
        scl  = I2C3_SCL_PIN;
        port = I2C3_SCL_GPIO_PORT;
    } else {
        sda  = I2C2_SDA_PIN;
        scl  = I2C2_SCL_PIN;
        port = I2C2_SCL_GPIO_PORT;
    }
    GPIO_InitStructure.GPIO_Pin   = scl;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &GPIO_InitStructure);

    while (GPIO_ReadInputDataBit(port, sda)) {
        GPIO_ToggleBits(port, scl);
    }

    // Configure the pins in alternate function mode
    GPIO_InitStructure.GPIO_Pin   = scl;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // open drain
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;  // Alternate function
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init( port, &GPIO_InitStructure );
}

/** ***************************************************************************
 * @name _i2c_error_irq()  LOCAL Generic error interrrupt handler
 * @brief Clear the selected I2C flag
 *
 * @param [in] i - index into gI2C global
 * @retval N/A
 ******************************************************************************/
static void _i2c_error_irq(int i)
{
    I2C_TypeDef *I2Cx = gI2C[i].I2Cx;

    gI2C[i].error = I2C_ReadRegister(I2Cx, I2C_Register_SR1) & 0xFF00;
    if(I2C_EVENT_SLAVE_ACK_FAILURE & I2C_GetLastEvent(I2Cx)) {
        I2Cx->CR1 |= I2C_CR1_STOP; /// Generate Stop condition

        /// Clear the selected I2C flag
        I2Cx->SR1 = (uint16_t)~( I2C_FLAG_AF & ( (uint32_t)0x00FFFFFF ) );
    }
    if( I2C_GetITStatus(I2Cx, I2C_IT_ARLO)) {
        // Generate clock signals so slave will release SDA
        _i2c_clear_arbitration_lost(i);
    }
    gI2C[i].state = i2cStateError;

    if (gI2C[i].callback != NULL) {
        gI2C[i].callback();
    }
}


/** ***************************************************************************
 * @name I2C1_ER_IRQHandler()  LOCAL Interrupt handlers, must be so named to
 *       receive interrupt, these get passed off to the generic handler
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void I2C1_ER_IRQHandler(void)
{
    OSDisableHook();
    _i2c_error_irq(kI2C1);
    OSEnableHook();
}

/** ***************************************************************************
 * @name I2C3_ER_IRQHandler()  LOCAL Interrupt handlers, must be so named to
 *       receive interrupt, these get passed off to the generic handler
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void I2C3_ER_IRQHandler(void)
{
    OSDisableHook();
    _i2c_error_irq(kI2C3);
    OSEnableHook();
}

void I2C2_ER_IRQHandler(void)
{
    OSDisableHook();
    _i2c_error_irq(kI2C2);
    OSEnableHook();
}


/*************************************************************************
* Function Name: _i2c_dma_irq
* Description: I2C DMA handler
* Parameters:  index into gI2C global.
*
* Return: none
**************************************************************************/
static void _i2c_dma_irq(int i)
{
    gI2C[i].state = i2cStateRxWaitStop;
    DMA_ITConfig( gI2C[i].dmaStream, DMA_IT_TC, DISABLE );
    _i2c_event(i);
}


/** ***************************************************************************
 * @name I2C1_DMA_RX_STREAM_IRQHANDLER()  LOCAL I2C1 Magnetomter, Temperature
 * @brief Specified as the DMA ISR handler in _I2C1_Init handler
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void I2C1_DMA_RX_STREAM_IRQHANDLER(void)
{
    OSDisableHook();
    // Check the status of the selected DMA Stream interrupt (transfer complete)
    if( DMA_GetITStatus( I2C1_DMA_RX_STREAM, I2C1_DMA_RX_INT_DONE ) ) {
        _i2c_dma_irq(kI2C1);
    }

    // Clear the DMA Stream's interrupt pending bits
    DMA_ClearITPendingBit( I2C1_DMA_RX_STREAM, I2C1_DMA_RX_INT_DONE );
    OSEnableHook();
}

/** ***************************************************************************
 * @name I2C1_DMA_RX_STREAM_IRQHANDLER()  LOCAL I2C3 Accelerometer
 * @brief Specified as the DMA ISR handler in _I2C3_Init handler
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void I2C3_DMA_RX_STREAM_IRQHANDLER(void)
{
    OSDisableHook();
    // Check the status of the selected DMA Stream interrupt (transfer complete)
    if( DMA_GetITStatus( I2C3_DMA_RX_STREAM, I2C3_DMA_RX_INT_DONE ) ) {
        _i2c_dma_irq(kI2C3);
    }

    // Clear the DMA Stream's interrupt pending bits
    DMA_ClearITPendingBit( I2C3_DMA_RX_STREAM, I2C3_DMA_RX_INT_DONE );
    OSEnableHook();
}

void I2C2_DMA_RX_STREAM_IRQHANDLER(void)
{
    OSDisableHook();
    // Check the status of the selected DMA Stream interrupt (transfer complete)
    if( DMA_GetITStatus( I2C2_DMA_RX_STREAM, I2C2_DMA_RX_INT_DONE ) ) {
        _i2c_dma_irq(kI2C2);
    }

    // Clear the DMA Stream's interrupt pending bits
    DMA_ClearITPendingBit( I2C2_DMA_RX_STREAM, I2C2_DMA_RX_INT_DONE );
    OSEnableHook();
}

/** ***************************************************************************
 * @name _i2c_event_irq() LOCAL Generic I2C state machine interrupt delayed
 * @brief service handler
 * @param [in] i - index into gI2C global. Start should be sent before this is
 *   called. State machine is modelled on the one described in
 *   http://www.st.com/st-web-ui/static/active/en/resource/technical/document/
           application_note/CD00209826.pdf
 * @retval N/A
 ******************************************************************************/
static void _i2c_event(int i)
{
    I2C_TypeDef    *I2Cx     = gI2C[i].I2Cx;
    uint32_t       lastEvent = I2C_GetLastEvent(I2Cx);
    enum eI2CState prevState = gI2C[i].state;

    if (gI2C[i].timeout) {
      gI2C[i].timeout--;
    }
    // Start Bit
    if (lastEvent & I2C_IT_SB) //I2C_IT_SB (0x02000001) I2Cx_EV_IRQHANDLER()
    {
        gI2C[i].state = i2cStateAddrWaitAck;
        if (gI2C[i].txSize) {
            I2C_Send7bitAddress(I2Cx,
                                gI2C[i].slaveAddr,
                                I2C_Direction_Transmitter);
        } else if (gI2C[i].rxSize) {
            I2C_Send7bitAddress(I2Cx,
                                gI2C[i].slaveAddr,
                                I2C_Direction_Receiver);
        } else {
            gI2C[i].state = i2cStateError;
        }
    } else if( (gI2C[i].state == i2cStateAddrWaitAck) && (lastEvent & I2C_IT_ADDR) ) {
        if (gI2C[i].txSize) {
            I2Cx->DR = *gI2C[i].txData; // send the data out

            gI2C[i].txData++;
            gI2C[i].txSize--;
            gI2C[i].state = i2cStateDataSend;
        } else if (gI2C[i].rxSize) {
            if (1 == gI2C[i].rxSize ) {
                I2Cx->CR1     |= I2C_CR1_STOP;
                /// Disable the acknowledgement
                I2Cx->CR1     &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
                gI2C[i].state = i2cStateRxData;
            } else { // gI2C[i].rxSize  >= 2
                gI2C[i].state           = i2cStateRxDataDMA;
                DMA_ClearITPendingBit(gI2C[i].dmaStream, gI2C[i].dmaFlags);
                gI2C[i].dmaStream->M0AR = (uint32_t) gI2C[i].rxData;
                gI2C[i].dmaStream->NDTR = gI2C[i].rxSize;
                gI2C[i].dmaStream->CR   |= (uint32_t)DMA_SxCR_EN; //DMA_Cmd( gI2C[i].dmaStream, ENABLE );

                /// Enable the DMA interrupt with the transfer complete interrupt mask (DMA_IT_TC)
                DMA_ITConfig( gI2C[i].dmaStream, DMA_IT_TC, ENABLE );

                I2Cx->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
            }
        } else {
            gI2C[i].state = i2cStateError;
        }
    } else if( (gI2C[i].state == i2cStateDataSend) &&
               ( lastEvent & I2C_IT_TXE ) ) { // transmit buff empty
        if(gI2C[i].txSize) {
            I2Cx->DR = *gI2C[i].txData; // send the data

            gI2C[i].txData++;
            gI2C[i].txSize--;
            gI2C[i].state = i2cStateDataSend;
        } else if (gI2C[i].rxSize) {
            /// Generate a START condition
            I2Cx->CR1     |= I2C_CR1_START;
            gI2C[i].state = i2cStateAddrSend;
        } else {
            /// all done with tx and there is no rx
            gI2C[i].state = i2cStateDataWaitAck;
        }
    } else if( (gI2C[i].state == i2cStateDataWaitAck)  &&
               (lastEvent & I2C_IT_BTF)) { // byte transfer finished
        I2Cx->CR1     |= I2C_CR1_STOP;
        gI2C[i].state = i2cStateDone; /// transmit complete

    } else if( ( gI2C[i].state == i2cStateRxData) &&
             ( (lastEvent & I2C_IT_BTF) || (lastEvent & I2C_IT_RXNE) ) ) { // byte transfer finihed, recieve not empty
        gI2C[i].rxSize--;
        if( 1 ==  gI2C[i].rxSize ) {
            I2Cx->CR1   |= I2C_CR1_STOP;
            /// Disable the acknowledgement
            I2Cx->CR1   &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
        }
        *gI2C[i].rxData = (uint8_t)I2Cx->DR;
        gI2C[i].rxData++;
        if( 0 == gI2C[i].rxSize ) {
            gI2C[i].state = i2cStateDone;
        }
    } else if (gI2C[i].state == i2cStateRxWaitStop) {
        gI2C[i].state     = i2cStateDone;
    } else {
        gI2C[i].thisEvent = lastEvent; // thisEvent not used
    }

    if (gI2C[i].timeout == 0) {
        gI2C[i].state = i2cStateDone;
        gI2C[i].error = ERROR_TIMEOUT;

        I2C_Cmd(gI2C[i].I2Cx, DISABLE);
        I2C_ITConfig(gI2C[i].I2Cx, I2C_IT_BUF |
                             I2C_IT_EVT       |
                             I2C_IT_ERR, DISABLE);
    }

    if (((gI2C[i].state == i2cStateDone)  && prevState != i2cStateDone) ||
        ((gI2C[i].state == i2cStateError) && prevState != i2cStateError)) {
        if (gI2C[i].callback != NULL) {
            gI2C[i].callback();
        }
    }
    gI2C[i].lastEvent = lastEvent; // not used
}


/** ***************************************************************************
 * @name _i2c_event_irq() LOCAL  Interrupt handlers, must be so named to receive
 *      interrupt, these get passed off to the generic handler
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void I2C1_EV_IRQHandler(void) {
    OSDisableHook();
    _i2c_event(kI2C1);
    OSEnableHook();
}

/** ***************************************************************************
 * @name I2C3_EV_IRQHandler() LOCAL  Interrupt handlers, must be so named to
 *       receive interrupt, these get passed off to the generic handler
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void I2C3_EV_IRQHandler(void) {
    OSDisableHook();
    _i2c_event(kI2C3);
    OSEnableHook();
}


void I2C2_EV_IRQHandler(void) {
    OSDisableHook();
    _i2c_event(kI2C2);
    OSEnableHook();
}


// FIXME: should be part of the driver not exposed to the user
/** ***************************************************************************
 * @name i2c_open()Init I2C bus open
 *
 * @param [in] I2Cx - which I2C channel to open
 * @param [in] callback - optional callback to call when I2C transfer is complete
 * @retval status TRUE = good
 ******************************************************************************/
uint8_t i2c_open (I2C_TypeDef* I2Cx,
                  void (*callback)())
{
    int i = _i2cx_to_index(I2Cx);

    if ( (gI2C[i].state != i2cStateDone)) {
//    if ( (gI2C[i].state != i2cStateDone) || !gI2C[i].notInUse) {
      //ERROR_INT("I2C OPEN error, state: ", gI2C[i].state);
//      ERROR_INT(" notInUse: ", gI2C[i].notInUse);
//ERROR_INT(" I2C[i] = ", i);
        //ERROR_ENDLINE();
        return FALSE;
    }

    gI2C[i].timeout  = I2C_TIMEOUT;
    gI2C[i].callback = callback;
    gI2C[i].notInUse = FALSE; // FIXME: needs to be atomic
    return TRUE;
}

// FIXME: should be part of the driver not exposed to the user
/** ***************************************************************************
 * @name i2c_close() API Init I2C interface release
 *
 * @param [in] I2Cx - which bus to close
 * @retval N/A
 ******************************************************************************/
void i2c_close (I2C_TypeDef* I2Cx)
{
    int i = _i2cx_to_index(I2Cx);
    gI2C[i].notInUse = TRUE; // FIXME: needs to be atomic
}


/** ***********************************************************************
* @name: I2C1_ConfigureDMA
* @param [in] which I2C channel
* @retval N/A
*************************************************************************/
void I2C1_ConfigureDMA( int i )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel            = I2C1_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)( &(I2C1->DR) );
    DMA_InitStructure.DMA_PeripheralInc      = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
    DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
    DMA_InitStructure.DMA_BufferSize         = 0;

    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_Init( I2C1_DMA_RX_STREAM, &DMA_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_DMA_RX_STREAM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C1_INTR_PRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = I2C1_INTR_SUBPRIO;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    gI2C[i].dmaFlags  = I2C1_DMA_RX_FLAGS;
    gI2C[i].dmaStream = I2C1_DMA_RX_STREAM;
}


/** ****************************************************************************
 * @name _i2c1_init() LOCAL Init I2C1 interface pins, clocks and interrupts
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
static  void _i2c1_init(void)
 {
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    DMA_Cmd( I2C1_DMA_RX_STREAM, DISABLE );
    RCC_AHB1PeriphClockCmd( I2C1_DMA_CLK, ENABLE );

    /// Enable the peripheral clock
    // SCL and SCA are on bus B
    RCC_AHB1PeriphClockCmd( I2C1_GPIO_CLK, ENABLE );

    /// Attach the pins to the peripheral alternate function
    GPIO_PinAFConfig( I2C1_SCL_GPIO_PORT,
                      I2C1_SCL_SOURCE,
                      GPIO_AF_I2C1 );
    GPIO_PinAFConfig( I2C1_SDA_GPIO_PORT,
                      I2C1_SDA_SOURCE,
                      GPIO_AF_I2C1 );

    /// Configure the pins in alternate function mode
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN;
    GPIO_Init( I2C1_SCL_GPIO_PORT, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = I2C1_SDA_PIN;
    GPIO_Init( I2C1_SDA_GPIO_PORT, &GPIO_InitStructure );

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    /// Program the Mode, duty cycle, Own address, Ack, Speed, and Acknowledged Address
    /// I2C configuration
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_ClockSpeed          = I2C1_SPEED;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    /// Enable the I2C Events Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C1_INTR_PRIO; // 6
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = I2C1_INTR_SUBPRIO; // 1
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /// Enable the I2C Errors Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    I2C1_ConfigureDMA(kI2C1);
 }

/** ****************************************************************************
 * @name: I2C3_ConfigureDMA Init I2C1 DMA
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void I2C3_ConfigureDMA( int i )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel            = I2C3_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)( &(I2C3->DR) );
    DMA_InitStructure.DMA_PeripheralInc      = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
    DMA_InitStructure.DMA_BufferSize         = 0;

    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_Init( I2C3_DMA_RX_STREAM, &DMA_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel                   = I2C3_DMA_RX_STREAM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C3_INTR_PRIO; // 4
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = I2C3_INTR_SUBPRIO; // 1
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    gI2C[i].dmaFlags  = I2C3_DMA_RX_FLAGS;
    gI2C[i].dmaStream = I2C3_DMA_RX_STREAM;
}



/** ***************************************************************************
 * @name _i2c3_init() LOCAL Init I2C3 pins clocks and iterrupts
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
static  void _i2c3_init(void)
 {
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    DMA_Cmd( I2C3_DMA_RX_STREAM, DISABLE );
    RCC_AHB1PeriphClockCmd( I2C3_DMA_CLK, ENABLE );

    RCC_AHB1PeriphClockCmd(I2C3_GPIO_SCL_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(I2C3_GPIO_SDA_CLK, ENABLE);

    GPIO_PinAFConfig(I2C3_SCL_GPIO_PORT, I2C3_SCL_SOURCE, GPIO_AF_I2C3);
    GPIO_PinAFConfig(I2C3_SDA_GPIO_PORT, I2C3_SDA_SOURCE, GPIO_AF_I2C3);

    GPIO_InitStructure.GPIO_Pin   = I2C3_SCL_PIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C3_SCL_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = I2C3_SDA_PIN;
    GPIO_Init(I2C3_SDA_GPIO_PORT, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);

    /// I2C configuration
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = I2C3_SPEED;
    I2C_Init(I2C3, &I2C_InitStructure);

    /// Enable the I2C Events Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C3_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C3_INTR_PRIO; // 4
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = I2C3_INTR_SUBPRIO; // 1
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /// Enable the I2C Errors Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = I2C3_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    I2C3_ConfigureDMA(kI2C3);
}



void I2C2_ConfigureDMA( int i )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel            = I2C2_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)( &(I2C2->DR) );
    DMA_InitStructure.DMA_PeripheralInc      = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
    DMA_InitStructure.DMA_BufferSize         = 0;

    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_Init( I2C2_DMA_RX_STREAM, &DMA_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel                   = I2C2_DMA_RX_STREAM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C2_INTR_PRIO; // 4
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = I2C2_INTR_SUBPRIO; // 1
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    gI2C[i].dmaFlags  = I2C2_DMA_RX_FLAGS;
    gI2C[i].dmaStream = I2C2_DMA_RX_STREAM;
}

static  void _i2c2_init(void)
 {
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    DMA_Cmd( I2C2_DMA_RX_STREAM, DISABLE );
    RCC_AHB1PeriphClockCmd( I2C2_DMA_CLK, ENABLE );

    RCC_AHB1PeriphClockCmd(I2C2_GPIO_CLK, ENABLE);

    GPIO_PinAFConfig(I2C2_SCL_GPIO_PORT, I2C2_SCL_SOURCE, GPIO_AF_I2C2);
    GPIO_PinAFConfig(I2C2_SDA_GPIO_PORT, I2C2_SDA_SOURCE, GPIO_AF_I2C2);

    GPIO_InitStructure.GPIO_Pin   = I2C2_SCL_PIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C2_SCL_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = I2C2_SDA_PIN;
    GPIO_Init(I2C2_SDA_GPIO_PORT, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    /// I2C configuration
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = I2C2_SPEED;
    I2C_Init(I2C2, &I2C_InitStructure);

    /// Enable the I2C Events Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C2_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C2_INTR_PRIO; // 4
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = I2C2_INTR_SUBPRIO; // 1
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /// Enable the I2C Errors Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    I2C2_ConfigureDMA(kI2C2);
}


/** ***************************************************************************
 * @name i2c_configure() API Initializes specific I2C interface with knowledge
 *       of the board
 * @brief I2C3 Magnetomter, Temperature
 *        I2C3 Accelerometer
 *
 * @param [in] I2Cx -  which I2C to init
 * @retval N/A
 ******************************************************************************/
void i2c_configure (I2C_TypeDef* I2Cx)
{
    int i = _i2cx_to_index(I2Cx);

    gI2C[i].I2Cx     = I2Cx;
    gI2C[i].notInUse = TRUE;
    gI2C[i].state    = i2cStateDone;

    I2C_Cmd(gI2C[i].I2Cx, DISABLE);
    I2C_DeInit(gI2C[i].I2Cx);

    if (I2C1 == I2Cx)  {
        _i2c1_init();
    } else if (I2C3 == I2Cx) {
        _i2c3_init();
    }

    if (I2C2 == I2Cx) {
        _i2c2_init();
    }

    /// Enable interrupts from module
    I2C_ITConfig( gI2C[i].I2Cx, I2C_IT_BUF |
                                I2C_IT_EVT |
                                I2C_IT_ERR, ENABLE );
    I2C_GenerateSTOP(I2Cx, ENABLE);
    I2C_Cmd(gI2C[i].I2Cx, ENABLE); /// I2C Peripheral Enable

}
/** ***************************************************************************
 * @name i2c_data_request() API rx some data
 *
 * @param [in] I2Cx - which I2C to init
 * @param [in] slave_addr-  slave address
 * @param [in] read_addr - write address
 * @param [in] rec_buffer - rx buffer
 * @param [in] num_bytes - rx buffer size
 * @retval N/A
 ******************************************************************************/
uint8_t i2c_data_request(I2C_TypeDef *I2Cx,
                         uint16_t    slave_addr,
                         uint8_t     read_addr,
                         uint8_t*    rec_buffer,
                         uint8_t     num_bytes)
{
    int i = _i2cx_to_index(I2Cx);

    __disable_irq();
    gI2C[i].error     = FALSE;
    gI2C[i].slaveAddr = slave_addr;
    gI2C[i].rxSize    = num_bytes;
    gI2C[i].rxData    = rec_buffer;

    // to rx bytes we also need to tx one byte to the unit, the address to read
    gI2C[i].txSize       = 1;
    gI2C[i].txDataMem[0] = read_addr;
    gI2C[i].txData       = gI2C[i].txDataMem;
    gI2C[i].state        = i2cStateAddrSend;

    I2Cx->CR1 |= I2C_CR1_ACK;   /// Enable the acknowledgement
    I2Cx->CR1 |= I2C_CR1_START; /// Generate a START condition
    __enable_irq();

    return TRUE;
}


/** ***************************************************************************
 * @name i2c_data_send() API tx data
 *
 * @param [in] I2Cx -  which I2C to use
 * @param [in] slave_addr- slave address
 * @param [in] send_buffer - tx buffer
 * @param [in] num_bytes - buffer size
 * @retval always return TRUE - forces context switch, holds at call for return
 ******************************************************************************/
uint8_t i2c_data_send(I2C_TypeDef *I2Cx,
                      uint16_t    slave_addr,
                      uint8_t     *send_buffer,
                      uint8_t     num_bytes)
{
    int i = _i2cx_to_index(I2Cx);
    __disable_irq();
    gI2C[i].error     = FALSE;
    gI2C[i].slaveAddr = slave_addr;
    gI2C[i].rxSize    = 0;

    /// to rx bytes we also need to tx one byte to the unit, the address to read
    gI2C[i].txSize = num_bytes;
    gI2C[i].txData = send_buffer;
    gI2C[i].state  = i2cStateAddrSend;
    I2Cx->CR1 |= I2C_CR1_ACK;   /// Enable the acknowledgement
    I2Cx->CR1 |= I2C_CR1_START; /// Generate a START condition
    __enable_irq();

    return TRUE;
}

/** ***************************************************************************
 * @name i2c_is_done() API check for completion
 *
 * @param [in] I2Cx -  which I2C to use
 * @retval TRUE if i2c transaction is complete TRUE or FALSE
 ******************************************************************************/
uint8_t i2c_is_done(I2C_TypeDef *I2Cx)
{
    int i = _i2cx_to_index(I2Cx);

    return (gI2C[i].state ==  i2cStateDone);
}

/** ****************************************************************************
 * @name i2c_has_error(void) API return error to external caller
 *
 * @param
 * @retval error value - FALSE if no error
 ******************************************************************************/
uint32_t i2c_has_error(I2C_TypeDef *I2Cx)
{
    int i = _i2cx_to_index(I2Cx);
    return (gI2C[i].error);
}
