/*******************************************************************************
* File Name          : led.h
* Author             : Daich
* Revision           : 1.0
* Date               : 11/10/2019
* Description        : led drive head file
*
* HISTORY***********************************************************************
* 11/10/2019  |                                             | Daich
* Description: create
* HISTORY***********************************************************************
* 11/10/2019  |                                             | Daich
* Description: modift LED_ON LED_OFF  LED(x,y)
*******************************************************************************/
#ifndef _LED_H_
#define _LED_H_
#include "driver.h"
#include "boardDefinition.h"

#define ON 		GPIO_PIN_SET
#define OFF 	GPIO_PIN_RESET
#define TOOGLE  2



#define LED_STATUS_CLK_ENABLE()                __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_RTCM_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_PPS_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()



#define LED_STATUS_PORT    	LED1_PORT
#define LED_STATUS_PIN     	LED1_PIN

#define LED_RTCM_PORT      	LED2_PORT
#define LED_RTCM_PIN       	LED2_PIN

#define LED_PPS_PORT       	LED3_PORT
#define LED_PPS_PIN        	LED3_PIN

#define LED_STATUS_ON       (HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET))
#define LED_STATUS_OFF      (HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET))
#define LED_STATUS_TOOGLE() (HAL_GPIO_TogglePin(LED_STATUS_PORT, LED_STATUS_PIN))
#define LED_STATUS(x)       ((x) ? HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET) : \
							HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET))

#define LED_PPS_ON          (HAL_GPIO_WritePin(LED_PPS_PORT, LED_PPS_PIN, GPIO_PIN_RESET))
#define LED_PPS_OFF         (HAL_GPIO_WritePin(LED_PPS_PORT, LED_PPS_PIN, GPIO_PIN_SET))
#define LED_PPS_TOOGLE()    (HAL_GPIO_TogglePin(LED_PPS_PORT, LED_PPS_PIN))
#define LED_PPS(x)          ((x) ? HAL_GPIO_WritePin(LED_PPS_PORT, LED_PPS_PIN, GPIO_PIN_RESET) : \
						    HAL_GPIO_WritePin(LED_PPS_PORT, LED_PPS_PIN, GPIO_PIN_SET))


#define LED_RTCM_ON         (HAL_GPIO_WritePin(LED_RTCM_PORT, LED_RTCM_PIN, GPIO_PIN_RESET))
#define LED_RTCM_OFF        (HAL_GPIO_WritePin(LED_RTCM_PORT, LED_RTCM_PIN, GPIO_PIN_SET))
#define LED_RTCM_TOOGLE()   (HAL_GPIO_TogglePin(LED_RTCM_PORT, LED_RTCM_PIN))
#define LED_RTCM(x)         ((x) ? HAL_GPIO_WritePin(LED_RTCM_PORT, LED_RTCM_PIN, GPIO_PIN_RESET) : \
						    HAL_GPIO_WritePin(LED_RTCM_PORT, LED_RTCM_PIN, GPIO_PIN_SET))

#define LED(x,y) ((y) ?  ((y==2)?x ##_TOOGLE():x ##_ON): x ##_OFF)

rtk_ret_e led_driver_install();
void   LED1_On(void);
void   LED2_On(void);
void   LED3_On(void);
void   LED1_Off(void);
void   LED2_Off(void);
void   LED3_Off(void);
void   LED1_Toggle(void);
void   LED2_Toggle(void);
void   LED3_Toggle(void);

#endif
