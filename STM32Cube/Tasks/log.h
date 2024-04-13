#ifndef __LOG
#define __LOG

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

// TODO: determine optimal numbers for these
#define MAX_MSG_LENGTH 32 // max length of log data string (bytes)
#define QUEUE_TIMEOUT_TICKS 50 // ticks to wait for sending/receiving queue

extern xQueueHandle logErrorQueue;
extern xQueueHandle logInfoQueue;
extern xQueueHandle logDebugQueue;

/**
 * Log Level
*/
typedef enum {
    LOG_LVL_ERROR,  // Errors (non-recoverable) and warnings are combined into this
    LOG_LVL_INFO,   // Info (data from sensors, etc)
    LOG_LVL_DEBUG,  // Only for debugging on the ground
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
 * Data to log, only used internally by log functions and queue
*/
typedef struct
{
    uint32_t timestamp;
    LogLevel_t level;
    LogDataSource_t dataSource;
    uint8_t* dataBuffer; // can contain empty chars; uart/sd will ignore those, only read up to len
    uint8_t dataLength; // will never exceed MAX_MSG_LENGTH
} LogData_t;

/**
 * Create queues, etc
*/
void logInit(void);

/**
 * Log an error-level message to error queue
*/
void logError(uint8_t* msg, LogDataSource_t source);

/**
 * Log an info-level message to info queue
*/
void logInfo(uint8_t* msg, LogDataSource_t source);

/**
 * Log a debug-level message to debug queue
*/
void logDebug(uint8_t* msg, LogDataSource_t source);

/**
 * FreeRTOS task for the logger
 * Continuously read queues, format data, and send to uart/SD
*/
void logTask(void *argument);


#ifdef __cplusplus
}
#endif

#endif
