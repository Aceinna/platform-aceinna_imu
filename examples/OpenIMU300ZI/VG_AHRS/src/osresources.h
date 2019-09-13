#ifndef _OS_RES_H
#define _OS_RES_H
#include "cmsis_os.h"
#include "portmacro.h"


#ifdef __MAIN

osSemaphoreDef(GYRO_READY_SEM);
osSemaphoreDef(ACCEL_READY_SEM);
osSemaphoreDef(MAG_READY_SEM);
osSemaphoreDef(TEMP_READY_SEM);
osSemaphoreDef(DATA_ACQ_SEM);
osSemaphoreDef(CLI_SEM);
osSemaphoreId gyroReadySem;
osSemaphoreId accelReadySem;
osSemaphoreId magReadySem;
osSemaphoreId tempReadySem;
osSemaphoreId dataAcqSem;
osSemaphoreId cliSem;

#else

extern osSemaphoreId gyroReadySem;
extern osSemaphoreId accelReadySem;
extern osSemaphoreId magReadySem;
extern osSemaphoreId tempReadySem;
extern osSemaphoreId dataAcqSem;
extern osSemaphoreId cliSem;

#endif


#endif
