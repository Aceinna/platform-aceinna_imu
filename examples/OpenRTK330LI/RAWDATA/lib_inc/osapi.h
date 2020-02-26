#ifndef _OS_API_H
#define _OS_API_H
#include "cmsis_os.h"
#include "portmacro.h"
#include <stdint.h>
#include "osapi.h"
#define   OSEnterISR() volatile int ulDummy = portSET_INTERRUPT_MASK_FROM_ISR()
#define   OSExitISR()  portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy )

extern void OSDisableHook();
extern void OSEnableHook();
extern void OS_Delay(uint32_t msec);
extern void OSDisableHookIfNotInIsr();
extern void OSEnableHookIfNotInIsr();
extern int gIsrDisableCount;

#define  getSystemTime()          osKernelSysTick()
#define  getSystemTimeAsDouble() (double)osKernelSysTick()/configTICK_RATE_HZ;



#define OS_INFINITE_TIMEOUT portMAX_DELAY
#define OS_NO_TIMEOUT 		0


// #define   OSEnterISR() do{ gIsrDisableCount++; __ASM  ("cpsid i");} while(0)
// #define   OSExitISR()  do{ gIsrDisableCount--; if (gIsrDisableCount == 0) { __ASM  ("cpsie i"); }}while(0)

#define   ENTER_CRITICAL() do{ gIsrDisableCount++; __ASM  ("cpsid i");} while(0)
#define   EXIT_CRITICAL()  do{ gIsrDisableCount--; if (gIsrDisableCount == 0) { __ASM  ("cpsie i"); }}while(0)

extern uint32_t SysTimer;

#endif
