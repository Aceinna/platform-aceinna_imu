/** ****************************************************************************
 * @file spi.c
 * @author jsun
 * @date: 2011-02-10 11:42:26 -0800 (Thu, 10 Feb 2011)
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * description: This SPI driver sets up DMA and interrupts to transfer data
 * back and forth across the buses. SP11 [MAX21000 gyro] and SPI3 [client] for
 * customer external communication.
 * @version 15866
 ******************************************************************************/
#include <stdint.h>
#include <salvo.h>

#include "stm32f2xx_conf.h"
#include "dmu.h"
#include "spi.h"
#include "boardDefinition.h"
#include "stm32f2xx_dma.h"
#include "stm32f2xx_spi.h"
#include "xbowsp_algorithm.h"

#include "debug.h" // serial console debug cmds
#include "UserCommunication_SPI.h" // with DMU380
#include "ucb_packet.h"

void (*gSPI1DMACompleteCallback)(void) = NULL;
void (*gSPI3DMACompleteCallback)(void) = NULL;
void  _spi3_DMA_Callback(void);

enum spi_state {
   READ_BYTE_1  = 0,
   READ_2_BYTES = 1,
   RESET_SPI    = 2
};

extern UcbPacketStruct gSpiUcbPacket; // taskDataacquisition.c

// max burst is 36 so 64 has headroom built in for future burst messages
uint8_t dummyBuffer[ 64 ] = {0};  // slave read (MASTER write)
int     gSPI1Complete = FALSE;
/** ****************************************************************************
 * @name: get_spi1_complete() API
 * @brief Used by Read/WriteMAX21000Register functions to detect DMA transfer
 *        complete.
 * @retval: DMA xfer complete status
 ******************************************************************************/
uint8_t get_spi1_complete() { return gSPI1Complete; }

/** ***************************************************************************
 * @name    _SPI1_Init() LOCAL
 * @brief The following function sets up the MASTER SPI1 [INTERNAL Sensors]
 *
 * @param [in] cpolAndCphaHigh -set cpol high or low
 * @retval success = 0
 ******************************************************************************/
uint8_t _SPI1_Init(uint8_t cpolAndCphaHigh)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    SPI_DeInit(SPI1);
    SPI_Cmd(SPI1, DISABLE);

    /// Set up reset and clock control (RCC)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    RCC_AHB1PeriphClockCmd(SPI1_GPIO_MOSI_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(SPI1_GPIO_MISO_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(SPI1_GPIO_SCK_CLK,  ENABLE);

    RCC_AHB1PeriphClockCmd(SPI1_DMA_CLK, ENABLE);

    /// Configure the SPI peripheral
    GPIO_PinAFConfig(SPI1_MOSI_PORT, SPI1_MOSI_SOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI1_MISO_PORT, SPI1_MISO_SOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI1_SCK_PORT,  SPI1_SCK_SOURCE,  GPIO_AF_SPI1);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN;
    GPIO_Init(SPI1_MOSI_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
    GPIO_Init(SPI1_MISO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
    GPIO_Init(SPI1_SCK_PORT, &GPIO_InitStructure);

    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

    if (cpolAndCphaHigh) {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    } else {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; /// software manages the NSS

    /// fPCLK / baudrate
    /// fPCLK is the APB1 clock frequency
    /// that is currenly SystemCoreClock which is 120MHz
    /// the MAX21000 can run at 10MHz
    /// but SPI mode starts slow and speeds up. ~0.5  Mhz
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;

    SPI_Init(SPI1, &SPI_InitStructure);

    /// DMA
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel            = SPI1_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
    DMA_InitStructure.DMA_BufferSize         = 0;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_Init(SPI1_DMA_RX_STREAM, &DMA_InitStructure);
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_Init(SPI1_DMA_TX_STREAM, &DMA_InitStructure);

    /// Set up DMA interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = SPI1_DMA_RX_STREAM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    SPI_Cmd(SPI1, ENABLE);

    return SPI_NO_ERROR;
}

/** ***************************************************************************
 * @name    _SPI3_Init() LOCAL
 * @brief The following function sets up the CLIENT SPI3 [EXTERNAL USER]
 *
 * @param [in] cpolAndCphaHigh -set cpol high or low
 * @retval success = 0
 ******************************************************************************/
uint8_t _SPI3_Init( uint8_t cpolAndCphaHigh )
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /// Disable the DMA streams (re-enabled in spi_transfer())
    DMA_Cmd( SPI3_DMA_RX_STREAM, DISABLE );
    DMA_Cmd( SPI3_DMA_TX_STREAM, DISABLE );

    /// ------ Enable the DMA controller clock for DMA1 (corresponding to SPI3) ------
    RCC_AHB1PeriphClockCmd( SPI3_DMA_CLK, ENABLE );

	/// ---------- Enable Peripheral (SPI3) Clock (connected to the DMA stream) ----------
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI3, ENABLE );
	RCC_APB1PeriphResetCmd( RCC_APB1Periph_SPI3, ENABLE );

	/// Enable GPIO clocks
	RCC_AHB1PeriphClockCmd( SPI3_GPIO_MOSI_CLK,    ENABLE );
	RCC_AHB1PeriphClockCmd( SPI3_GPIO_MISO_CLK,    ENABLE );
	RCC_AHB1PeriphClockCmd( SPI3_GPIO_SCK_CLK,     ENABLE );
	RCC_AHB1PeriphClockCmd( SPI3_SLAVE_SELECT_CLK, ENABLE );

	SPI_Cmd( SPI3, DISABLE ); /// Disable SPI3 during configuration
	SPI_DeInit( SPI3 );       /// Reset SPI parameters to their default

	/// ---------- SPI GPIO Configuration ----------
	/// Connect pins to AF6 (GPIO alternate function)
	GPIO_PinAFConfig( SPI3_MOSI_PORT, SPI3_MOSI_SOURCE, GPIO_AF_SPI3 );
	GPIO_PinAFConfig( SPI3_MISO_PORT, SPI3_MISO_SOURCE, GPIO_AF_SPI3 );
	GPIO_PinAFConfig( SPI3_SCK_PORT,  SPI3_SCK_SOURCE,  GPIO_AF_SPI3 );
	GPIO_PinAFConfig( SPI3_SLAVE_SELECT_PORT, SPI3_SLAVE_SELECT_SOURCE, GPIO_AF_SPI3 );

	/// Configure GPIO pins
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;     /// input/output/alt func/analog
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// push-pull or open-drain
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;   /// Up/Down/NOPULL
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed

	GPIO_InitStructure.GPIO_Pin   = SPI3_MOSI_PIN;        /// MOSI C12
	GPIO_Init( SPI3_MOSI_PORT, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin   = SPI3_MISO_PIN;        /// MISO C11
	GPIO_Init( SPI3_MISO_PORT, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin   = SPI3_SCK_PIN;         /// SCK C10
	GPIO_Init( SPI3_SCK_PORT, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin  = SPI3_SLAVE_SELECT_PIN; /// nSS A15
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init( SPI3_SLAVE_SELECT_PORT, &GPIO_InitStructure);

	/// ---------- SPI GPIO Configuration ----------
	SPI_StructInit( &SPI_InitStructure );
	SPI_InitStructure.SPI_Mode      = SPI_Mode_Slave;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize  = SPI_DataSize_8b; /// Bits per transfer
	SPI_InitStructure.SPI_FirstBit  = SPI_FirstBit_MSB; /// Transmit MSB first

	/// CPOL/CPHA
	if ( cpolAndCphaHigh ) {
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; }
	else {
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; }

	/// nSS software/hardware mode
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;

	/// fPCLK / baudrate
	/// fPCLK is the APB1 clock frequency, which is currently SystemCoreClock
    /// (120MHz). The MAX21000 can run at 10MHz - SPI mode starts slow and
    /// speeds up. ~0.5  Mhz
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;

	/// Initialize the SPI communication protocol as specified above
	SPI_Init( SPI3, &SPI_InitStructure );

	/// Load the data register so the TXE interrupt doesn't trip at startup
	SPI3->DR = 0x00;

	SPI_Cmd( SPI3, ENABLE ); /// -------- Re-enable the SPI peripheral --------

    /// ------ 3) Program the DMA configuration ------
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel            = SPI3_DMA_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)( &(SPI3->DR) );
	DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
    // DMA_DIR would be here - (direction for the channel)
	DMA_InitStructure.DMA_BufferSize         = 0;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte; // shouldn't this be short?
// DMA_Mode = DMA_Mode_Normal
	DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
// DMA_FIFOMode = DMA_FIFOMode_Disable
// DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
// DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
// DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
// DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_Init( SPI3_DMA_RX_STREAM, &DMA_InitStructure );

	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init( SPI3_DMA_TX_STREAM, &DMA_InitStructure );

    /// ------ 4) Enable DMA interrupts ------
    /// Set up DMA interrupt to trigger after receiving the required number of
    ///   bytes from the master (1, 2, or 16 bytes)
    NVIC_InitStructure.NVIC_IRQChannel                   = SPI3_DMA_RX_STREAM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3; // was 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /// ------ 5) Enable the DMA stream ------
    /// Start the SPI DMA - DMA IRQ is triggered after the 8th bit is clocked in.
    ///   The interrupt reads cleans up from the prior DMA interrupt and issues
    ///   the read/write action based on the contents of the first byte.
    spi3_transfer( gUserSpi.writeBuffer,
                   dummyBuffer,
                   _1_BYTE  );

    return SPI_NO_ERROR;
}

void SPI3_DMA_TX_STREAM_IRQHANDLER( void )
{ // doesn't get hit
}

/** ***************************************************************************
 * @name spi_configure() API configure a selected SPI bus
 * @brief Configure the SPI interface via the initialization functions
 *        (defined above) in addition to the arguments passed to this function
 *        (used to set CPOL/CHPA and callback, which defines the callback
 *        function that executes upon DMA complete)
 *
 * @param [in] SPIx - which bus to set up
 * @param [in] cpolAndCphaHigh -
 * @param [in] callback function (not used so NULL function this version)
 * @retval success = 0, error = 1
 ******************************************************************************/
uint8_t spi_configure(SPI_TypeDef* SPIx,
					  uint8_t      cpolAndCphaHigh,
					  void         (*callback)(void))
{
    if( SPIx == SPI1 ) { // rate sensor SPI interface
        gSPI1DMACompleteCallback = callback; // Gyro has a callback
        return _SPI1_Init(cpolAndCphaHigh);
    } else if( SPIx == SPI3 ) { // user-communication external SPI interface
        gSPI3DMACompleteCallback = callback;   // NULL callback
        return _SPI3_Init( cpolAndCphaHigh );
    }
    gAlgorithm.bitStatus.comBIT.bit.spiError = 1;
    return SPI_ERROR_GENERIC;
}

/** ***************************************************************************
 * @name SPI1_DMA_RX_STREAM_IRQHANDLER()
 * @brief Specified as the DMA ISR handler in _SPI1_Init (SENSOR bus)
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void SPI1_DMA_RX_STREAM_IRQHANDLER(void)
{
    OSDisableHook();

    /// Gyro DMA Stream transfer complete interrupt
    if( DMA_GetITStatus( SPI1_DMA_RX_STREAM, SPI1_DMA_RX_INT_DONE ) )
    {
        /// receive buffer not empty flag
        if ( SPI_I2S_GetFlagStatus( kGyroSPI, SPI_I2S_FLAG_RXNE ) == EMPTY ) {
            gSPI1Complete = TRUE; // transfer complete flag for gyro driver
            gSPI1DMACompleteCallback(); // gyro callback loaded at init
            /// Disable the DMA Stream interrupt
            DMA_ITConfig( SPI1_DMA_RX_STREAM, DMA_IT_TC, DISABLE );
        }
    }

    /// DMA Stream's interrupt pending bits
    DMA_ClearITPendingBit( SPI1_DMA_RX_STREAM, SPI1_DMA_RX_INT_DONE );
    OSEnableHook();
}

/** ***************************************************************************
 * @name SPI3_DMA_RX_STREAM_IRQHANDLER()
 * @brief Specified as the DMA handler in _SPI3_Init (external USER COMM bus)
 *        all of this gets handled (cleared/reset) in the callback fcn
 * @param [in] N/A
 * @retval N/A
 * @brief moved state machine to _spi3_DMA_Callback()
 ******************************************************************************/
void SPI3_DMA_RX_STREAM_IRQHANDLER(void)
{   /// user Com DMA Stream interrupt (transfer complete)
    if( DMA_GetITStatus( SPI3_DMA_RX_STREAM, SPI3_DMA_RX_INT_DONE ) )
    {
        OSDisableHook();
        /// ========= Clean up the DMA transfer begun previously ==========
        /// Disable the Streams
        SPI3_DMA_RX_STREAM->CR &= ~(uint32_t)DMA_SxCR_EN;
        SPI3_DMA_TX_STREAM->CR &= ~(uint32_t)DMA_SxCR_EN;

        /// Disable the SPI3 DMA Stream interrupts
        DMA_ITConfig( SPI3_DMA_RX_STREAM, DMA_IT_TC, DISABLE );
        DMA_ITConfig( SPI3_DMA_TX_STREAM, DMA_IT_TC, DISABLE );

        /// Disable the SPI/DMA interface - spi3_transfer() enables these
        SPI3->CR2 &= (uint16_t)~( SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx );

        /// Clear the DMA Stream's interrupt pending bits
        DMA_ClearITPendingBit( SPI3_DMA_RX_STREAM, SPI3_DMA_RX_INT_DONE );
        DMA_ClearITPendingBit( SPI3_DMA_TX_STREAM, SPI3_DMA_TX_INT_DONE );
        /// ========= End of clean up ==========
    }
    _spi3_DMA_Callback();
    OSEnableHook();  /// Re-enable interrupts
}

/** ****************************************************************************
 * @name spi3_DMA_Callback() LOCAL state machine to handle external SPI USER COM
 * @brief SPI 3 is a SLAVE so all actions are initiated from a external MASTER
 * device all actions start here - requested read or write from the MASTER
 * device.
 * @param [in] N/A
 * @retval N/A - void function - no context switch so doesn't hold the CPU in
 * the ISR. The ISR already stopped the interrupts for the DMA - spi3_transfer()
 * re-enables them
 * @details the system is armed with a request for a single byte,
 * on write a second byte is set up for slave to receive the MOSI payload byte
 * then the system is re-armed with a request for a single byte.
 * MISO reads may be one or more with the data interlaced so for each read
 * request there needs 8 clocks for the payload but the next read address may
 * be sent on the payload clocks. So the read straddles the current payload and
 * possible next address request. When address is zero then the system sets up
 * a final 2 byte transfer with clocks for the last byte sent out and and a byte
 * to re-arm the system for the next transfer request from the master.
 ******************************************************************************/
void _spi3_DMA_Callback(void)
{ // FIXME: this could be a more standard state machine
    static uint8_t reArmAfterWrite;
    static volatile uint8_t postRead = 0;
    static uint8_t addressIndex;   // byte 0 write address, byte 1 read or stop
    static uint8_t writeDataFlag;
//    static uint8_t processWrite = 0;
    uint8_t        writeData;
    uint8_t        registerAddr;
    uint8_t tempBuffer[256];  // holds the values sent during a burst-read (not needed for operation)

    if ( reArmAfterWrite ) { // arm after a write - read arm is in post burst
        reArmAfterWrite = false;
        addressIndex    = 0;
        // set up for next command from the MASTER
        spi3_transfer( gUserSpi.writeBuffer,
                       gUserSpi.zeroBuffer,
                       _1_BYTE );
        writeData = gUserSpi.writeBuffer[0];
        // Handle the write
        // temp valueS to prevent the order of the volatile access from issuing a warning
         registerAddr = gUserSpi.registerAddr;
         HandleRegisterWrite_UserSPI( registerAddr, writeData );
    } else {
        // After interrupt from the DMA that data has arrived, decode the address
        //   byte to determine whether the master is sending a write or read request.
        gUserSpi.registerAddr  = gUserSpi.writeBuffer[addressIndex] & 0x7F;
        writeDataFlag = gUserSpi.writeBuffer[addressIndex] & 0x80;

        if( postRead ) {
            // finish the last 8 clocks of a read then arm the system
            // with the next 8 awaiting next data transmission
            spi3_transfer(gUserSpi.writeBuffer,
                          &( gUserSpi.DataRegister[gUserSpi.registerAddr] ),
                          _2_BYTES );
            postRead = false;
        } else { // regular reads / writes, get the MOSI payload or send MISO
            if( writeDataFlag ) { // write MOSI
                spi3_transfer( gUserSpi.writeBuffer, // get payload byte
                               gUserSpi.zeroBuffer,
                               _1_BYTE );
                reArmAfterWrite = true; // after payload interrupt re-arm
                postRead = false;
            } else { // read MISO
                if( gUserSpi.registerAddr == 0x3E ) { // JD scaled data
                    // A JD burst-read - Send out 16 bytes MISO
                    //    then issue the DMA interrupt.
//                    spi3_transfer(gUserSpi.writeBuffer,
//                                  gUserSpi.BurstRegister,
//                                  _16_BYTES );  // pre canned size
                    spi3_transfer(tempBuffer,
                                  gUserSpi.BurstRegister,
                                  _16_BYTES );  // pre canned size
                } else if ( (gUserSpi.registerAddr > SPI_REG_JD_BURST_READ) && // 0x3f
                            (gUserSpi.registerAddr <= SPI_REG_N0_BURST_READ) ) { // to 0x45
                   if ( gUserSpi.registerAddr != gSpiUcbPacket.spiAddress ) {
                        gSpiUcbPacket.spiAddress = gUserSpi.registerAddr;
                       // On a message switch need to send out the correct number
                       // of bytes so send out zeros first time
                       // can change this to load like the BIT words into
                       // a set of buffers so change happens at the request
                       if( gUserSpi.registerAddr == 0x3f ) {
                           // F1
                           gSpiUcbPacket.payloadLength = 54;
                       } else if( gUserSpi.registerAddr == 0x40 ) {
                           // F2
                           gSpiUcbPacket.payloadLength = 66;
                       } else if( gUserSpi.registerAddr == 0x41 ) {
                           // S0
                           gSpiUcbPacket.payloadLength = 30;
                       } else if( gUserSpi.registerAddr == 0x42 ) {
                           // S1
                           gSpiUcbPacket.payloadLength = 24;
                       } else if( gUserSpi.registerAddr == 0x43 ) {
                           // A1
                           gSpiUcbPacket.payloadLength = 32;
                       }

                        /// SPI Burst MASTER MISO READs Switch address comand ->
                        spi3_transfer( tempBuffer,
                                       gUserSpi.zeroBuffer, // zero's (loaded in send_packet.c)
                                       gSpiUcbPacket.payloadLength ); // variable size
                    } else {
                        /// SPI normal Burst MASTER MISO READs ->
                        spi3_transfer( tempBuffer,
                                       gSpiUcbPacket.payload,
                                       gSpiUcbPacket.payloadLength ); // variable size
                    }
                } else {
                    // Reset the register variables after a read occurs
                    if( gUserSpi.registerAddr == 0x74 ) {
                        // orientation registers (reset orientation counter/flags after read)
                        gUserSpi.firstByte = 0;
                        gUserSpi.byteCntr = 0;
                    }

                    // standard read. Send out 2 bytes [payload, payload/next address]
                    //   to the master device, issue a DMA interrupt.
                    spi3_transfer( gUserSpi.writeBuffer,
                                   &( gUserSpi.DataRegister[gUserSpi.registerAddr] ),
                                   _2_BYTES ); // non-burst possible multiple MISO
                }
                postRead = true; // After reads are complete, finish the missing 8 bits and re-arm
            }
            addressIndex = 1;
        }
    }
}

/** ****************************************************************************
 * @name: spi1_transfer() API Gyro Spi interface  for data transfer
 * resets the DMA to restart for the next transfer
 * @param [in] in - pointer to rx buffer,  length long
 * @param [in] out - pointer to tx buffer, length long
 * @param [in] length - number of bytes to send
 * @retval: always 0
 ******************************************************************************/
uint8_t spi1_transfer( volatile uint8_t *in,
                       volatile uint8_t *out,
                       uint16_t         length )
{
    /// Clear all DMA Stream's pending flag on the TX and RX stream
    DMA2->LIFCR = (uint32_t)( SPI1_DMA_TX_FLAGS & (uint32_t)0x0F7D0F7D );
    DMA2->LIFCR = (uint32_t)( SPI1_DMA_RX_FLAGS & (uint32_t)0x0F7D0F7D );

    /// Configure the memory address: DMA_MemoryTargetConfig()
    SPI1_DMA_RX_STREAM->M0AR = (uint32_t)in;
    SPI1_DMA_TX_STREAM->M0AR = (uint32_t)out;

    /// number of bytes to be transferred on the DMA
    SPI1_DMA_RX_STREAM->NDTR = (uint16_t)length;
    SPI1_DMA_TX_STREAM->NDTR = (uint16_t)length;

    /// Enable the DMA Stream
    SPI1_DMA_RX_STREAM->CR |= (uint32_t)DMA_SxCR_EN;
    SPI1_DMA_TX_STREAM->CR |= (uint32_t)DMA_SxCR_EN;

    gSPI1Complete = FALSE;

    /// Enable the DMA interrupt with the transfer complete interrupt mask
    DMA_ITConfig( SPI1_DMA_RX_STREAM, DMA_IT_TC, ENABLE );

    SPI1->CR2 |= ( SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx ); /// Enable the SPI/DMA interface
    return 0;
}


/** ****************************************************************************
 * @name: spi3_transfer() API
 * @brief Description:
 *		Used for bidirectional transfer of data on the customer user COM bus.
 *      Commands extracted from DMA functions to improve performance.  resets
 *      the DMA to restart for the next transfer
 *
 * @param [in] in - pointer to rx buffer,  length long
 * @param [in] out - pointer to tx buffer, length long
 * @param [in] length - number of bytes to send
 * @retval: always 0
 ******************************************************************************/
uint8_t spi3_transfer( volatile uint8_t *in,
                       volatile uint8_t *out,
                       uint16_t         length )
{
    /// pending flags on the streams
    DMA_ClearFlag( SPI3_DMA_TX_STREAM, SPI3_DMA_TX_FLAGS );
    DMA_ClearFlag( SPI3_DMA_RX_STREAM, SPI3_DMA_RX_FLAGS );

    /// Configure the memory address
    SPI3_DMA_RX_STREAM->M0AR = (uint32_t)in;
    SPI3_DMA_TX_STREAM->M0AR = (uint32_t)out;

    // Writes the number of data units (length) to be transferred on the DMA
    SPI3_DMA_RX_STREAM->NDTR = (uint16_t)length;
    SPI3_DMA_TX_STREAM->NDTR = (uint16_t)length;

    /// Enable the DMA Stream
    SPI3_DMA_RX_STREAM->CR |= (uint32_t)DMA_SxCR_EN;
    SPI3_DMA_TX_STREAM->CR |= (uint32_t)DMA_SxCR_EN;

    /// Enable the DMA interrupt with the transfer complete interrupt mask
    DMA_ITConfig( SPI3_DMA_RX_STREAM, DMA_IT_TC, ENABLE );

    SPI3->CR2 |= ( SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx ); // Enable the SPI/DMA

    return 0;
}
/** ****************************************************************************
 * @name: spi_set_baud() API change the baud rate of the selected spi bus
 * @brief SPI_BaudRatePrescaler_xxx are in stm32f2xx_spi.h
 *         with clock at 120Hz, this is ~8Mhz
 *         baud rate is clock/(2*prescaler)
 *         SPI_BaudRatePrescaler_8    ~7.5 MHz
 *         SPI_BaudRatePrescaler_16   ~3.75 MHz
 *         SPI_BaudRatePrescaler_32   ~1.875 MHz
 *         SPI_BaudRatePrescaler_64   ~1    Mhz
 *         SPI_BaudRatePrescaler_128  ~0.5  Mhz
 *         SPI_BaudRatePrescaler_256  ~0.25 Mhz
 * @param [in] SPIx - SPI channel selected
 * @param [in] baudRatePrescaler - the bus speed to set
 * @retval: 0 if success
 ******************************************************************************/
void spi_set_baud(SPI_TypeDef* SPIx,
                  uint16_t     baudRatePrescaler)
{
    uint16_t       tmpreg = 0;
    const uint16_t baudRateMask = SPI_BaudRatePrescaler_256;

    __disable_irq();
    SPI_Cmd( SPIx, DISABLE );

    tmpreg    = SPIx->CR1;
    tmpreg    &= ~baudRateMask;
    tmpreg    |= baudRatePrescaler;
    SPIx->CR1 = tmpreg;

    SPI_Cmd( SPIx, ENABLE );
    __enable_irq();
}
