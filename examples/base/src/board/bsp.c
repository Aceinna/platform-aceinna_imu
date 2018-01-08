/******************************************************************************
* @file bsp.c
* @brief
* File description:
*		Use the file to Configure Cortex M3
*
* $Rev: 16166 $
* @date: 2011-03-09 11:53:45 -0800 (Wed, 09 Mar 2011) $
* @author: whpeng $
******************************************************************************/
#include "stm32f2xx_conf.h"
#include "bsp.h"

#ifdef FL // disable led 
/** ****************************************************************************
 * @name led_on
 * @brief  Turns the selected LED on.
 *
 * Trace:
 *
 * @param [in] led - which one to turn on
 * @retval N/A
 ******************************************************************************/
void led_on(Led_TypeDef led)
{
	STM_EVAL_LEDOn(led);
}


/** ****************************************************************************
 * @name led_off
 * @brief  Turns the selected LED off.
 *
 * Trace:
 *
 * @param [in] led - which one to turn off
 * @retval N/A
 ******************************************************************************/
void led_off(Led_TypeDef led)
{
	STM_EVAL_LEDOff(led);
}

/** ****************************************************************************
 * @name led_toggle
 * @brief  Toggles the selected LED.
 *
 * Trace:
 *
 * @param [in] led - which one to toggle
 * @retval N/A
 ******************************************************************************/
void led_toggle(Led_TypeDef led)
{
	STM_EVAL_LEDToggle(led);
}

#endif //FL
/** ****************************************************************************
 * @name BSP_init
 * @brief This function should be called by your application code before you make
 *        use of any of the functions found in this module
 *
 * Trace:
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void BSP_init(void)
{
//    RCC_config(); /// System clocks configuration
	//NVIC_config();
	GPIO_config();
	InitSystemTimer();

}

/** ****************************************************************************
 * @name GPIO_config
 * W@brief configuration GPIO
 *
 * Trace:
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void GPIO_config(void)
{
  ;
}

/** ****************************************************************************
 * @name RCC_config
 * @brief Configures the different system clocks and Enable peripherals.
 *
 * Trace:
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void RCC_config(void)
{
#define SYSTICK_RCC_CONF
#if defined(SYSTICK_RCC_CONF) /// temporary reserves
	ErrorStatus HSEStartUpStatus;

	RCC_DeInit(); /// RCC system reset(for debug purpose)
	RCC_HSEConfig(RCC_HSE_ON); /// Enable HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); /// Wait till HSE is ready

	if(HSEStartUpStatus == SUCCESS)
	{
		/// Enable Prefetch Buffer
        FLASH_PrefetchBufferCmd(ENABLE);
        FLASH_InstructionCacheCmd(ENABLE);
        FLASH_DataCacheCmd(ENABLE);

		FLASH_SetLatency(FLASH_Latency_3);
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);

        //RCC_PLLConfig( RCC_PLLSource_HSE, 25, 240, 2, 5 );
        RCC_PLLConfig( RCC_PLLSource_HSE, 24, 232, 2, 5 );

		RCC_PLLCmd(ENABLE); /// Enable PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) /// Wait till PLL is ready
		{ /* spin */ }

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); /// Select PLL as system clock

		/// Wait till PLL is used as system clock source
		while(RCC_GetSYSCLKSource() != 0x08)
		{ /* spin */ }
	}
// (FIXME) JSM - HSE enable in the following function
   SystemCoreClockUpdate();
#elif defined(SYSTICK_STM32_CONF)
	SystemInit(); /// temporary reserves
#endif

}

/** ****************************************************************************
 * @name NVIC_config
 * @brief Configures Vector Table base location.
 *
 * Trace:
 *
 * @param N/A
 * @retval
 ******************************************************************************/
void NVIC_config(void)
{
#if defined (VECT_TAB_RAM)
	/// Set the Vector Table base location at 0x20000000
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#elif defined(VECT_TAB_FLASH_IAP)
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, NVIC_FLASH_IAP);
#else  /// VECT_TAB_FLASH
	/// Set the Vector Table base location at 0x08000000
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
	/// Configure the NVIC Preemption Priority Bits
}

