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

#define LOG_FILE_NAME_SIZE 500

extern const char *logsPath;
extern char logFileName[LOG_FILE_NAME_SIZE];

#ifdef __cplusplus
}
#endif


#endif
