#include <stdarg.h>
#include "string.h"

#include "printf.h"
#include "otits.h"

#include "sdmmc.h"
#include "log.h"

/**
 * Buffer holding one block of log msgs
*/
typedef struct log_buffer {
    int currMsgNum;
    SemaphoreHandle_t currMsgNumMutex;
    char buffer[LOG_BUFFER_SIZE];
    bool isFull;
} log_buffer;

static log_buffer logBuffers[NUM_LOG_BUFFERS];
static int CURRENT_BUFFER = 0;
static SemaphoreHandle_t logWriteMutex;

static int droppedMsgs = 0;
static int fullBuffMoments = 0;
static int logWriteTimeouts = 0;
static int invalidRegionMoments = 0;
static int critErrs = 0;
static int noFullBuffMoments = 0;

// Queue of full buffers ready for output. Typically a max of `n - 1` buffers will be in queue as
// the nth buffer is being dumped. But leave space for all `n` buffers in case queueReceive is
// delayed. Otherwise, CURRENT_BUFFER can get stuck on a full buffer that's outside the queue.
QueueHandle_t fullBuffersQueue;

// OTITS TESTS
Otits_Result_t test_logInfo() {
	Otits_Result_t res;
	if (!logInfo("otits test", "otits!")){
		res.info = "logInfo fail";
		res.outcome = TEST_OUTCOME_FAILED;
		return res;
	}
	res.info = "";
	res.outcome = TEST_OUTCOME_PASSED;
	return res;
}

Otits_Result_t test_currentBufferFull() {
	Otits_Result_t res;
	if (logBuffers[CURRENT_BUFFER].isFull){
		res.info = "all log buffers full";
		res.outcome = TEST_OUTCOME_FAILED;
		return res;
	}
	res.info = "";
	res.outcome = TEST_OUTCOME_PASSED;
	return res;
}

bool logInit(void)
{
    fullBuffersQueue = xQueueCreate(NUM_LOG_BUFFERS, sizeof(log_buffer*));
    logWriteMutex = xSemaphoreCreateMutex();

    if (fullBuffersQueue == NULL || logWriteMutex == NULL) {
        return false;
    }

    for (int i = 0; i < NUM_LOG_BUFFERS; i++) {
        logBuffers[i].isFull = false;
        logBuffers[i].currMsgNum = 0;
        logBuffers[i].currMsgNumMutex = xSemaphoreCreateMutex();
    }
    otitsRegister(test_currentBufferFull, TEST_SOURCE_LOGGER, "CurrBufFull");
    otitsRegister(test_logInfo, TEST_SOURCE_LOGGER, "LogInfo");
    return true;
}

/**
 * @brief Construct a log msg, send to buffer, and handle switching buffers when full.
 * @note This method should only be called internally from the log functions
 * @param source 
 * @param level
 * @param msg a formatted msg
 * @param msgArgs variable list of arguments for the msg format. Use variable arguments here so that log
 * users don't have to manually do sprintf_ beforehand - instead, this function will handle formatting the string.
 * Should be safe since this is essentially a sprintf_ wrapper, where variable arguments are acceptable.
*/
bool logGeneric(const char* source, LogLevel_t level, const char* msg, va_list msgArgs) {
    // get timestamp immediately so the following mutex wait time is irrelevant
    TickType_t timestamp = xTaskGetTickCount();

    // protect this section of calculating which buffer is being used, must be done one at a time.
    if (xSemaphoreTake(logWriteMutex, 10) != pdPASS) {
        logWriteTimeouts++;
        droppedMsgs++;
        return false;
    }

    // get current buffer
    log_buffer* currentBuffer = &logBuffers[CURRENT_BUFFER];

    // if current buffer is full, then all of them must be full. ERROR!!! do not proceed
    if (currentBuffer->isFull) {
        xSemaphoreGive(logWriteMutex);
        return false;
    }

    // get the current msg number
    int currMsgNum = currentBuffer->currMsgNum++;

    // if this is the last msg of the buffer, rotate immediately so another task can start with a fresh buffer
    if (currMsgNum == MSGS_PER_BUFFER - 1) {
        // set isFull before sending to queue for extra protection
        currentBuffer->isFull = true;
        CURRENT_BUFFER = (CURRENT_BUFFER + 1) % NUM_LOG_BUFFERS;
    }

    // check that we didn't get an invalid region to write
    if (currMsgNum >= MSGS_PER_BUFFER) {
        invalidRegionMoments++;
        droppedMsgs++;
        return false;
    }

    // we now have a safe region to write in so the rest can be done concurrently without mutex protection
    xSemaphoreGive(logWriteMutex);

    // format and write the log msg
    int charsWritten = 0;
    // move to the chunk of space that this msg owns in this buffer
    char* msgStart = currentBuffer->buffer + currMsgNum * MAX_MSG_LENGTH;

    charsWritten += snprintf_(msgStart + charsWritten, MAX_MSG_LENGTH - charsWritten, "%c: [%d] %s ", level, (int) timestamp, source);
    if (charsWritten >= MAX_MSG_LENGTH) {
        charsWritten = MAX_MSG_LENGTH; // -1 to ignore the '\0' that snprintf automatically counts
    }

    charsWritten += vsnprintf_(msgStart + charsWritten, MAX_MSG_LENGTH - charsWritten, msg, msgArgs);
    if (charsWritten >= MAX_MSG_LENGTH) {
        charsWritten = MAX_MSG_LENGTH; // -1 to ignore the '\0' that snprintf automatically counts
    }

    // move to the last char in this msg's chunk of space, and set it to \n
    currentBuffer->buffer[currMsgNum * MAX_MSG_LENGTH + MAX_MSG_LENGTH - 1] = '\n';

    // check if this msg is the last one in the buffer (-1 because zero-indexing)
    if (currMsgNum == MSGS_PER_BUFFER - 1) {
        // if full, send this buffer to output queue and rotate to next buffer
        if (xQueueSendToBack(fullBuffersQueue, &currentBuffer, 0) != pdPASS) {
            // if queue is full, all buffers are already in queue. how did we get here ??? ERROR!!!!
            critErrs++;
            return false;
        }
    } else if (currMsgNum > MSGS_PER_BUFFER - 1) {
        // wtf we are NOT supposed to exceed the max number of msgs !!!!
        critErrs++;
    }

    return true;
}

// ----------------------------------------------------------------------------
// public log functions
// ----------------------------------------------------------------------------

bool logError(const char* source, const char* msg, ...) {
    va_list args;
    va_start(args, msg);
    bool success = logGeneric(source, LOG_LVL_ERROR, msg, args);
    va_end(args);
    return success;
}

bool logInfo(const char* source, const char* msg, ...) {
    va_list args;
    va_start(args, msg);
    bool success = logGeneric(source, LOG_LVL_INFO, msg, args);
    va_end(args);
    return success;
}

bool logDebug(const char* source, const char* msg, ...) {
#ifdef DEBUG
    va_list args;
    va_start(args, msg);
    bool success = logGeneric(source, LOG_LVL_DEBUG, msg, args);
    va_end(args);
    return success;
#endif
}

// ----------------------------------------------------------------------------

void logTask(void *argument) {
    log_buffer* bufferToPrint;
    // initalize log file stuff
    FATFS fs;
    FRESULT res = FR_OK;

    res |= f_mount(&fs, "", 0);

    res |= f_mkdir(logsPath);

    initUniqueLogFileName();

    FIL logfile;
    res |= f_open(&logfile, logFileName, FA_CREATE_ALWAYS);

    res |= f_close(&logfile);


    if (res != FR_OK && res != FR_EXIST) {
        // flag init error
    }

    // wait for a full buffer to appear in the queue; timeout is long - queues are not expected to fill up super quickly
    for (;;) {
		if (xQueueReceive(fullBuffersQueue, &bufferToPrint, 1000000) == pdPASS) {
				FRESULT result = FR_OK;
                result |= f_open(&logfile, logFileName, FA_OPEN_APPEND | FA_WRITE);
                // print entire buffer for max efficiency and prevent data loss in case file closing fails
                result |= f_write(&logfile, bufferToPrint->buffer, LOG_BUFFER_SIZE, NULL);
                result |= f_close(&logfile);
            	// uart print for testing
            	// !!!! Ensure the timeout (rn 3000) is long enough to transmit a whole log chunk !!!
            	// HAL_UART_Transmit(&huart4, bufferToPrint->buffer, bufferToPrint->index, 3000);

                // don't need mutex here - anyone who tries to acquire this log will get prevented by isFull=true
                memset(bufferToPrint->buffer, 0, LOG_BUFFER_SIZE);
                bufferToPrint->currMsgNum = 0;
                bufferToPrint->isFull = false;
        } else {
            noFullBuffMoments++;
        }
        logInfo("log", "%d %d %d %d %d %d", droppedMsgs, fullBuffMoments, logWriteTimeouts, invalidRegionMoments, critErrs, noFullBuffMoments);
    }
}
