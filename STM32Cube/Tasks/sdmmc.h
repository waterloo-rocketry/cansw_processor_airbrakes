#ifndef __SDMMC
#define __SDMMC

#include <stdbool.h>

#include "printf.h"

#include "stm32h7xx_hal.h"

// function declarations
void initUniqueLogFileName();
bool sdmmcInit();

// variable declarations

#define LOG_FILE_NAME_SIZE 19 // LOGS/2147483647.txt

extern const char *logsPath;
extern char logFileName[LOG_FILE_NAME_SIZE];

#endif
