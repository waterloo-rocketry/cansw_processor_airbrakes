#ifndef __SDMMC
#define __SDMMC

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ff.h"

// function declarations
void initUniqueLogFileName();

// variable declarations
extern const char *logsPath;
extern char logFileName[500];

#ifdef __cplusplus
}
#endif


#endif
