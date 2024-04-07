#ifndef __SDMMC
#define __SDMMC

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ff.h"
#include <stdio.h>

// function declarations
void sdmmcTask(void* arguments);


#ifdef __cplusplus
}
#endif


#endif
