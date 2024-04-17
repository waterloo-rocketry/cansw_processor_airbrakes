#include "log.h"
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart4;

static struct log_buffer logBuffers[NUM_LOG_BUFFERS];
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

    for (int i = 0; i < NUM_LOG_BUFFERS; i++)
    {
        logBuffers[i].mutex = xSemaphoreCreateMutex();
        logBuffers[i].index = 0;
        logBuffers[i].isFull = false;
    }

    return true;
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
void logGeneric(LogDataSource_t source, LogLevel_t level, const char* msg, va_list msgArgs)
{
    // get timestamp immediately so the following mutex wait time is irrelevant
    TickType_t timestamp = xTaskGetTickCount();

    // there can only be 1 log writer at once
    if (xSemaphoreTake(logWriteMutex, 50) != pdPASS)
    {
        // timed out while waiting to log - maybe too many tasks are waiting to log. this should never happen?
    }
    else
    {
        // get the buffer here (only after taking writeMtx) since it's possible for a buffer to fill up while waiting for the writeMtx
        log_buffer* currentBuffer = &logBuffers[CURRENT_BUFFER];
    
    // if current buffer is full, all of them are full which should never happen. this will block loggers until one is emptied
    if (currentBuffer->isFull)
    {
        xSemaphoreGive(logWriteMutex);
        return;
    }
        
        // format and append the default log header. Limit snprintf to `MAX_MSG_LENGTH - 1` to leave room for \n at the end
        // TODO: make actually readable formatting for lvl and source
        int headerLength = snprintf(currentBuffer->buffer + currentBuffer->index, MAX_MSG_LENGTH - 1, "%d: [%ld][%d]", level, timestamp, source);
        currentBuffer->index += headerLength;

        // limit this to `MAX_MSG_LENGTH - headerLength - 1` to account for the header and the \n at the end
        currentBuffer->index += vsnprintf(currentBuffer->buffer + currentBuffer->index, MAX_MSG_LENGTH - headerLength - 1, msg, msgArgs);

        // add \n in here instead of asking the sender to do it. This guarantees \n in case msg gets cut off
        currentBuffer->buffer[currentBuffer->index] = '\n';
        currentBuffer->index += 1;

        // check if space left in buffer can be guaranteed to fit another msg (which could be up to MAX_MSG_LENGTH chars)
        if (LOG_BUFFER_SIZE - currentBuffer->index > MAX_MSG_LENGTH) 
        {
        // set this before, in case queue is full. then it prevents from writing at least
        currentBuffer->isFull = true;

            // if full, send this buffer to output queue and move to next empty one
            if (xQueueSendToBack(fullBuffersQueue, &currentBuffer, 0) != pdPASS)
            {
                // if queue does not have space, all n - 1 buffers are full which should not be possible!! ERROR!
            }
            CURRENT_BUFFER = (CURRENT_BUFFER + 1) % NUM_LOG_BUFFERS;
        }
        
        xSemaphoreGive(logWriteMutex);
    }
}

// ----------------------------------------------------------------------------
// public log functions
// ----------------------------------------------------------------------------

void logError(const LogDataSource_t source, const char* msg, ...)
{
    va_list args;
    va_start(args, msg);
    logGeneric(source, LOG_LVL_ERROR, msg, args);
    va_end(args);
}

void logInfo(const LogDataSource_t source, const char* msg, ...)
{
    va_list args;
    va_start(args, msg);
    logGeneric(source, LOG_LVL_INFO, msg, args);
    va_end(args);
}

void logDebug(const LogDataSource_t source, const char* msg, ...)
{
    // TODO: wrap this in a if/else for if debugging is enabled
    va_list args;
    va_start(args, msg);
    logGeneric(source, LOG_LVL_DEBUG, msg, args);
    va_end(args);
}

// ----------------------------------------------------------------------------

void logTask(void *argument)
{
    log_buffer* bufferToPrint;

    // wait for a full buffer to appear in the queue; timeout is long - queues are not expected to fill up super quickly
    for (;;)
    {
		if (xQueueReceive(fullBuffersQueue, &bufferToPrint, 1000000) == pdPASS)
        {
            // TODO: do uart transmit better (use _IT?)
            // buffers fill from 0, so `index` conveniently indicates how many chars of data there are to print
			HAL_UART_Transmit(&huart4, bufferToPrint->buffer, bufferToPrint->index, 100);

            bufferToPrint->index = 0;
        }
    }
}