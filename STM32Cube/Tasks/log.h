#ifndef __LOG
#define __LOG

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"


// TODO: determine optimal numbers for these
/* max length of log data string (bytes) */
#define MAX_MSG_LENGTH 128
/* size of one log buffer (bytes) */
#define LOG_BUFFER_SIZE 4096
/* number of log buffers */
#define NUM_LOG_BUFFERS 3

/**
 * Log Level
*/
typedef enum {
    LOG_LVL_ERROR = 'E',  // Errors (non-recoverable) and warnings are combined into this
    LOG_LVL_INFO = 'I',   // Info (data from sensors, etc)
    LOG_LVL_DEBUG = 'D',  // Only for debugging on the ground
} LogLevel_t;

/**
 * Log data source
*/
typedef enum
{
    SOURCE_FLIGHT_EVENT,
    SOURCE_HEALTH,
    SOURCE_CAN_RX,
    SOURCE_CAN_TX,
    SOURCE_SENSOR,
    SOURCE_STATE_EST,
    SOURCE_APOGEE_PRED,
    SOURCE_EXT_TARGET    
} LogDataSource_t;

/**
 * Buffer holding one block of log msgs
*/
typedef struct log_buffer {
    SemaphoreHandle_t mutex;
    uint16_t index;
    char buffer[LOG_BUFFER_SIZE];
    bool isFull;
} log_buffer;

/**
 * Create buffers, mutexes, etc
*/
bool logInit(void);

/**
 * Log an error-level message
*/
void logError(const LogDataSource_t source, const char* msg, ...);

/**
 * Log an info-level message
*/
void logInfo(const LogDataSource_t source, const char* msg, ...);

/**
 * Log a debug-level message
*/
void logDebug(const LogDataSource_t source, const char* msg, ...);

/**
 * FreeRTOS task for the logger
 * Wait on buffers to become full, then dump buffer to SD/uart
*/
void logTask(void *argument);


#ifdef __cplusplus
}
#endif

#endif
