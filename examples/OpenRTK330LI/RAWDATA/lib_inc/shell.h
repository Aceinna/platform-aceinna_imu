/*******************************************************************************
* File Name          : shell.h
* Author             : daich
* Revision           : 1.0
* Date               : 
* Description        : shell.h
*
* HISTORY***********************************************************************
* 10/12/2019  |                                             | Daich
* Description: created
* HISTORY***********************************************************************
* 16/12/2019  |                                             | Daich
* Description: add marco    SHELL_TASK
*******************************************************************************/

#ifndef _SHELL_H_
#define _SHELL_H_
#include "stdint.h" 
#include "GlobalConstants.h"        
#include "stdio.h"


#define SHELL_ENABLE        false
#define SHELL_TASK          false
#define SHELL_RX_MAX        (256+32)        
#define SHELL_TX_MAX        (512)          

bool is_cmd_right(void * buffer,void * cmd);   
void shell_service(void); 
void debug_uart_handle(void);
void ConsoleTask(void const *argument);
void parse_debug_cmd();
int bt_uart_parse(uint8_t* bt_buff);
#endif