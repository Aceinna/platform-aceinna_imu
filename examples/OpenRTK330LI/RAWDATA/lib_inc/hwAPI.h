/**
******************************************************************************
* @file    hwAPI.h 
******************************************************************************
*/
#ifndef __HW_API_H
#define __HW_API_H

#include "stdint.h"
#include "GlobalConstants.h"


// GPIO - related fucntions
void    HW_SetTestMode(BOOL fOn);
BOOL    HW_IsTestOK();

// system related functions
void    HW_SystemReset(void);
void    HW_JumpToApp();
void    HW_EnforceBootMode();

extern BOOL fSPI;
extern BOOL fUART;


#endif //__UART_H