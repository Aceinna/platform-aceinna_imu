/**
  ******************************************************************************
  * @file    SPI/SPI_TwoBoards/DataExchangeInterrupt/stm32f2xx_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_conf.h"

#include "boardDefinition.h"
#include "Indices.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "gyroscope.h"

#include "salvo.h"

/** @addtogroup STM32F2xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup SPI_DataExchangeInterrupt
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef SPI_SLAVE
 extern  __IO uint8_t RxBuffer[];
 extern __IO uint8_t Rx_Idx;
#endif
#ifdef SPI_MASTER
 extern __IO uint8_t CmdTransmitted;
 extern __IO uint8_t Tx_Idx;
 extern __IO uint8_t CmdStatus;
#endif

extern uint8_t TxBuffer[];
extern __IO uint32_t TimeOut;
__IO uint8_t Counter = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

#if defined(__ICCARM__)
// Copies of register values pushed onto stack before invoking HardFault handler
volatile struct sHardFaultStacked {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} HardFault;

// Copy HardFault register values from stack to debugger-friendly memory locations
static void save_fault_cpu_state(uint32_t* stackPtr)
{
   HardFault.r0 = stackPtr[0];
   HardFault.r1 = stackPtr[1];
   HardFault.r2 = stackPtr[2];
   HardFault.r3 = stackPtr[3];
   HardFault.r12 = stackPtr[4];
   HardFault.lr = stackPtr[5];
   HardFault.pc = stackPtr[6];
   HardFault.psr = stackPtr[7];

}

#define IAR_FUNC_ENTRY_PUSHES 2

#endif // __ICCARM__



/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

#if defined(__ICCARM__)
    // NOTE: this only works with IAR
    if (__get_CONTROL() & 2)
    {
        // Process SP in use.
        save_fault_cpu_state((uint32_t *) __get_PSP() + IAR_FUNC_ENTRY_PUSHES);
    }
    else
    {
        // Main SP in use.
        save_fault_cpu_state((uint32_t *) __get_MSP() + IAR_FUNC_ENTRY_PUSHES);
    }
#endif // __ICCARM__

    /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}


/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

void EXTI1_IRQHandler(void)
{
	OSDisableHook();
	
	setAccelI2CBusy(BUSY);
	AccelerometerDataReadyIRQ();

	//Clear the pending bit, write register directly
	//Replace EXTI_ClearITPendingBit(ACCEL_DATA_READY_EXTI_LINE);
	EXTI->PR = ACCEL_DATA_READY_EXTI_LINE;
	
	OSEnableHook();

}

// for gUserSpi
#include "UserCommunication_SPI.h"

// Handle the magnetometer and accelerometer interrupts by checking the individual interrupt status
void EXTI9_5_IRQHandler(void)
{
    OSDisableHook();

    if( EXTI_GetITStatus( ACCEL_DATA_READY_EXTI_LINE ) == SET ) {
        setAccelI2CBusy( BUSY );
        AccelerometerDataReadyIRQ();
        EXTI->PR = ACCEL_DATA_READY_EXTI_LINE;  // EXTI_ClearITPendingBit(ACCEL_DATA_READY_EXTI_LINE);
    }

    if( EXTI_GetITStatus( MAG_DATA_READY_EXTI_LINE ) == SET ) {
        MagnetomterDataReadyIRQ();
        EXTI->PR = MAG_DATA_READY_EXTI_LINE;   // EXTI_ClearITPendingBit(MAG_DATA_READY_EXTI_LINE);
    }

    OSEnableHook();
}


/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPIx_IRQHANDLER(void)
{
#ifdef SPI_SLAVE
  /* SPI in Slave Receiver mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_RXNE) == SET)
  {
    RxBuffer[Rx_Idx++] = SPI_I2S_ReceiveData(SPIx);
  }
#endif
#ifdef SPI_MASTER
  /* SPI in Master Tramitter mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_TXE) == SET)
  {
    if (CmdStatus == 0x00)
    {
	    /* Send Transaction code */
      SPI_I2S_SendData(SPIx, CmdTransmitted);
      CmdStatus = 0x01;
    }
    else
    {
      if (Tx_Idx < GetVar_NbrOfData())
      {
	      /* Send Transaction data */
        SPI_I2S_SendData(SPIx, TxBuffer[Tx_Idx++]);
      }
      else
      {
        /* Disable the Tx buffer empty interrupt */
        SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, DISABLE);
      }
    }
  }
#endif /* SPI_SLAVE */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
