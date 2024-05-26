#include "sdmmc.h"
#include "log.h"

#include <stdio.h>
#include <stdarg.h>


extern UART_HandleTypeDef huart4;
/*extern const char* logsPath;*/
/*extern char* logFileName;*/
/**/
/*extern initUniqueLogFileName()*/

static log_buffer logBuffers[NUM_LOG_BUFFERS];
static int CURRENT_BUFFER = 0; // TODO: better way to store current buffer than literally a global var
static SemaphoreHandle_t logWriteMutex;

// Queue of full buffers ready for output. Length of `n - 1` because all `n` buffers can never be full at once.
// Most extreme case: `n - 1` buffers are in queue, and the `nth` buffer is currently being dumped to output (already
// removed from the queue and protected by mutex, so it can't be written to until the dump is done)
QueueHandle_t fullBuffersQueue;

bool logInit(void)
{
    fullBuffersQueue = xQueueCreate(NUM_LOG_BUFFERS - 1, sizeof(log_buffer*));
    logWriteMutex = xSemaphoreCreateMutex();

    if (fullBuffersQueue == NULL || logWriteMutex == NULL) {
        return false;
    }

    for (int i = 0; i < NUM_LOG_BUFFERS; i++)
    {
        logBuffers[i].mutex = xSemaphoreCreateMutex();
        logBuffers[i].index = 0;
        logBuffers[i].isFull = false;

        if (logBuffers[i].mutex == NULL) {
            return false;
        }
    }

    return true;
}

/**
 * internal helper to convert source enum to string
*/
static const char* sourceToString(LogDataSource_t source)
{
    switch (source)
    {
        case SOURCE_FLIGHT_EVENT:
            return "FlightEvt";
            break;
        case SOURCE_HEALTH: 
            return "Health";
            break;
        case SOURCE_CAN_RX:
            return "CanRx";
            break;
        case SOURCE_CAN_TX:
            return "CanTx";
            break;
        case SOURCE_SENSOR:
            return "Sensor";
            break;
        case SOURCE_STATE_EST:
            return "StateEst";
            break;
        case SOURCE_APOGEE_PRED:
            return "TrajPred";
            break;
        case SOURCE_EXT_TARGET :
            return "ExtTarget";
            break;
        default:
            return "";
    }
}

/**
 * @brief Construct a log msg, send to buffer, and handle switching buffers when full.
 * @note This method should only be called internally from the log functions
 * @param source 
 * @param level
 * @param msg a formatted msg
 * @param msgArgs variable list of arguments for the msg format. Use variable arguments here so that log
 * users don't have to manually do sprintf beforehand - instead, this function will handle formatting the string.
 * Should be safe since this is essentially a sprintf wrapper, where variable arguments are acceptable.
*/
bool logGeneric(LogDataSource_t source, LogLevel_t level, const char* msg, va_list msgArgs)
{
    // get timestamp immediately so the following mutex wait time is irrelevant
    TickType_t timestamp = xTaskGetTickCount();

    // there can only be 1 log writer at once
    if (xSemaphoreTake(logWriteMutex, 50) != pdPASS)
    {
        return false;
        // timed out while waiting to log - maybe too many tasks are waiting to log. this should never happen?
    }
    
    // get the buffer here (only after taking writeMtx) since it's possible for buffers to rotate while waiting for the writeMtx
    log_buffer* currentBuffer = &logBuffers[CURRENT_BUFFER];
    
    // if current buffer is full, then all of them must be full. ERROR!!! do not proceed
    if (currentBuffer->isFull)
    {
        xSemaphoreGive(logWriteMutex);
        return false;
    }

    // TODO: still not sure if this is necessary since isFull exists, but doesnt hurt (unless needs to be optimized out later idk) 
    if (xSemaphoreTake(currentBuffer->mutex, 0) != pdPASS)
    {
        return false;
    }
    
    // format and append the default log header
	int headerLength = snprintf(currentBuffer->buffer + currentBuffer->index, MAX_MSG_LENGTH, "%c: [%d] %s ", level, (int) timestamp, sourceToString(source));
    currentBuffer->index += headerLength;

    // limit the actual msg to `MAX_MSG_LENGTH - headerLength` to account for the header we just printed
    int msgLength = vsnprintf(currentBuffer->buffer + currentBuffer->index, MAX_MSG_LENGTH - headerLength, msg, msgArgs);
    
    // snprintf behaviour moment: it returns a larger number than the limit if it had to truncate
	if (msgLength >= MAX_MSG_LENGTH - headerLength)
    {
        currentBuffer->index += MAX_MSG_LENGTH - headerLength - 1; // -1 cuz snprintf makes the last char \0
    }
    else
    {
        currentBuffer->index += msgLength; // no -1 here cuz snprintf normal return value doesn't count \0
    }

    // add \n in here instead of asking the sender to do it. This guarantees \n in all logs in case msg gets cut off
    currentBuffer->buffer[currentBuffer->index] = '\n';
    currentBuffer->index += 1;

    // if buffer cannot be guaranteed to fit another msg, rotate buffers (which can be up to `MAX_MSG_LENGTH + \n` chars)
    if (LOG_BUFFER_SIZE - currentBuffer->index < MAX_MSG_LENGTH + 1)
    {
        // do this before sending to queue in case queue is full. this prevents other loggers from writing to it
        currentBuffer->isFull = true;

        // if full, send this buffer to output queue and move to next empty one
        if (xQueueSendToBack(fullBuffersQueue, &currentBuffer, 0) != pdPASS)
        {
            // if queue does not have space, all n - 1 buffers are full which should not be possible!! ERROR!
            return false;
        }
        else
        {
            // rotate only if the queue write was successful. otherwise, stay on this buffer until queue hopefully empties
            CURRENT_BUFFER = (CURRENT_BUFFER + 1) % NUM_LOG_BUFFERS;
        }
    }
    
    xSemaphoreGive(currentBuffer->mutex);
    xSemaphoreGive(logWriteMutex);

    return true;
}

// ----------------------------------------------------------------------------
// public log functions
// ----------------------------------------------------------------------------

bool logError(const LogDataSource_t source, const char* msg, ...)
{
    va_list args;
    va_start(args, msg);
    bool success = logGeneric(source, LOG_LVL_ERROR, msg, args);
    va_end(args);
    return success;
}

bool logInfo(const LogDataSource_t source, const char* msg, ...)
{
    va_list args;
    va_start(args, msg);
    bool success = logGeneric(source, LOG_LVL_INFO, msg, args);
    va_end(args);
    return success;
}

bool logDebug(const LogDataSource_t source, const char* msg, ...)
{
#ifdef DEBUG
    va_list args;
    va_start(args, msg);
    bool success = logGeneric(source, LOG_LVL_DEBUG, msg, args);
    va_end(args);
    return success;
#endif
}

// ----------------------------------------------------------------------------

void logTask(void *argument)
{
	// initalize log file stuff
	FATFS fs;
	(void)f_mount(&fs, "", 0);

	(void)f_mkdir(logsPath);

	(void)initUniqueLogFileName();

	FIL logfile;
	(void)f_open(&logfile, logFileName, FA_CREATE_ALWAYS);
	(void)f_close(&logfile);

    log_buffer* bufferToPrint;

    // wait for a full buffer to appear in the queue; timeout is long - queues are not expected to fill up super quickly
    for (;;)
    {
		if (xQueueReceive(fullBuffersQueue, &bufferToPrint, 1000000) == pdPASS)
        {
            if (xSemaphoreTake(bufferToPrint->mutex, 0) == pdPASS)
            {
                // TODO: do uart transmit better
                // buffers fill from 0, so `index` conveniently indicates how many chars of data there are to print

            	(void)f_open(&logfile, logFileName, FA_OPEN_APPEND | FA_WRITE);
            	(void)f_write(&logfile, bufferToPrint->buffer, bufferToPrint->index, NULL);
            	(void)f_close(&logfile);

                bufferToPrint->index = 0;
                bufferToPrint->isFull = false;    
                xSemaphoreGive(bufferToPrint->mutex);            
            }
        }
    }
}
