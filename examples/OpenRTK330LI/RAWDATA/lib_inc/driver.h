/*******************************************************************************
* File Name          : driver.h
* Author             : Daich
* Revision           : 1.0
* Date               : 10/10/2019
* Description        : drive head file
*
* HISTORY***********************************************************************
* 10/10/2019  |                                             | Daich
* HISTORY***********************************************************************
* 16/12/2019  |                                             | Daich
* Description: add return type RTK_JSON
* 06/01/2020  |                                             | Daich
* Description: add BT_CMD BT_BASE function
*******************************************************************************/
#ifndef _DRIVER_H_
#define _DRIVER_H_
//#pragma once
#include "GlobalConstants.h"

typedef enum rtk_ret_e_
{
	RTK_FAIL = 0,
	RTK_OK = 1,
    RTK_SEM_OK = 2,
	UART_ERR = 10,
    RTK_JSON = 100,
	BT_CMD = 101,
    BT_BASE = 102,
}rtk_ret_e;

//typedef int bool;

#endif