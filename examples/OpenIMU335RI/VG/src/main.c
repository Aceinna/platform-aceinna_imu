#include "configurationAPI.h"
#include "calibrationAPI.h"
#include "taskDataAcquisition.h"
#include "EcuSettings.h"
#include "halAPI.h"
#include "sensorsAPI.h"
#include "osapi.h"
#include "bitAPI.h"

#ifndef UNIT_TEST
/**********************************************
* @brief 
* 
* @return int 
***********************************************/
int  main(void)
{
    
    uint32_t   rate;
    uint16_t  chipMask; 

    BIT_Iinitialize();
    ApplyFactoryConfiguration();
    HW_Init();  
    cal_Init();
    LoadEcuSettings();
    BIT_UpdateConfiguredSensorChips();

    OS_StartTimers(1000U);

    sens_Init(&chipMask);
    chipMask = (uint16_t)~chipMask & 0x07U;
    BIT_UpdateSensorSelfTestStatus((uint8_t)chipMask);

    rate = (uint32_t)config_GetBaudRate();
    UART_Init(USER_SERIAL_PORT, rate);

    TaskDataAcquisition();
    
    // should never return here
    while(rate){};

    return 1;
}

#endif //UNIT_TEST