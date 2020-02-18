#ifndef _OS_RES_H
#define _OS_RES_H

#ifndef BAREMETAL_OS
#include "cmsis_os.h"
#include "portmacro.h"
#endif



#ifdef __MAIN

osSemaphoreDef(GYRO_READY_SEM);
osSemaphoreDef(ACCEL_READY_SEM);
osSemaphoreDef(MAG_READY_SEM);
osSemaphoreDef(TEMP_READY_SEM);
osSemaphoreDef(NAV_DATA_READY_SEM);
osSemaphoreDef(CLI_READY_SEM);
osSemaphoreDef(DATA_ACQ_SEM);
osSemaphoreDef(CAN_DATA_SEM);
osSemaphoreDef(CLI_SEM);
osSemaphoreDef(RTK_START_SEM);
osSemaphoreDef(RTK_FINISH_SEM);

osSemaphoreId gyroReadySem;
osSemaphoreId accelReadySem;
osSemaphoreId magReadySem;
osSemaphoreId tempReadySem;
osSemaphoreId navDataReadySem;
osSemaphoreId cliReadySem;
osSemaphoreId dataAcqSem;
osSemaphoreId canDataSem;
osSemaphoreId cliSem;
osSemaphoreId RTKStartSem;
osSemaphoreId RTKFinishSem;
#else

#ifndef BAREMETAL_OS
extern osSemaphoreId gyroReadySem;
extern osSemaphoreId accelReadySem;
extern osSemaphoreId magReadySem;
extern osSemaphoreId tempReadySem;
extern osSemaphoreId navDataReadySem;
extern osSemaphoreId dataAcqSem;
extern osSemaphoreId canDataSem;
extern osSemaphoreId cliSem;
extern osSemaphoreId RTKStartSem;
extern osSemaphoreId RTKFinishSem;
#endif



#endif


#endif
