#include "sdmmc.h"
#include "log.h"

#include <stdarg.h>
#include "printf.h"
#include "otits.h"

static log_buffer logBuffers[NUM_LOG_BUFFERS];
static int CURRENT_BUFFER = 0;
static SemaphoreHandle_t logWriteMutex;

static int droppedMsgs = 0;
static int fullBuffMoments = 0;
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
    fullBuffersQueue = xQueueCreate(NUM_LOG_BUFFERS - 1, sizeof(log_buffer*));
    logWriteMutex = xSemaphoreCreateMutex();

    if (fullBuffersQueue == NULL || logWriteMutex == NULL) {
        return false;
    }

    for (int i = 0; i < NUM_LOG_BUFFERS; i++) {
        logBuffers[i].index = 0;
        logBuffers[i].isFull = false;
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

    // there can only be 1 log writer at once
    if (xSemaphoreTake(logWriteMutex, 50) != pdPASS) {
        droppedMsgs++;
        return false;
        // timed out while waiting to log - maybe too many tasks are waiting to log. this should never happen?
    }
    
    // get the buffer here (only after taking writeMtx) since it's possible for buffers to rotate while waiting for the writeMtx
    log_buffer* currentBuffer = &logBuffers[CURRENT_BUFFER];
    
    // if current buffer is full, then all of them must be full. ERROR!!! do not proceed
    if (currentBuffer->isFull) {
        fullBuffMoments++;
        droppedMsgs++;
        xSemaphoreGive(logWriteMutex);
        return false;
    }

    // format and append the log header and msg
    int charsWritten = 0;
    char* msgStart = currentBuffer->buffer + currentBuffer->index;

    charsWritten += snprintf_(msgStart + charsWritten, MAX_MSG_LENGTH - charsWritten, "%c: [%d] %s ", level, (int) timestamp, source);
    if (charsWritten >= MAX_MSG_LENGTH) {
        charsWritten = MAX_MSG_LENGTH; // -1 to ignore the '\0' that snprintf automatically counts
    }

    charsWritten += vsnprintf_(msgStart + charsWritten, MAX_MSG_LENGTH - charsWritten, msg, msgArgs);
    if (charsWritten >= MAX_MSG_LENGTH) {
        charsWritten = MAX_MSG_LENGTH; // -1 to ignore the '\0' that snprintf automatically counts
    }

    // add \n in here instead of asking the sender to do it. This guarantees \n in all logs in case msg gets cut off
    currentBuffer->buffer[currentBuffer->index + charsWritten] = '\n';
    currentBuffer->index += charsWritten + 1; // + 1 for the '\n'

    // check if this buffer can fit another msg of up to `MAX_MSG_LENGTH + '\n'` chars
    if (LOG_BUFFER_SIZE - currentBuffer->index <= MAX_MSG_LENGTH) {
        // set isFull before sending to queue in case queue is full. this prevents other loggers from writing to it
        currentBuffer->isFull = true;

        // if full, send this buffer to output queue and rotate to next buffer
        if (xQueueSendToBack(fullBuffersQueue, &currentBuffer, 0) != pdPASS) {
            // if queue is full, all buffers are already in queue. how did we get here ??? ERROR!!!!
            critErrs++;
            xSemaphoreGive(logWriteMutex);
            return false;
        }

        CURRENT_BUFFER = (CURRENT_BUFFER + 1) % NUM_LOG_BUFFERS;
    }

    xSemaphoreGive(logWriteMutex);
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
		if (xQueueReceive(fullBuffersQueue, &bufferToPrint, 10000) == pdPASS) {
				FRESULT result = FR_OK;
			    result |= f_open(&logfile, logFileName, FA_OPEN_APPEND | FA_WRITE);
                // buffers fill from 0, so `index` conveniently indicates how many chars of data there are to print
			    result |= f_write(&logfile, bufferToPrint->buffer, bufferToPrint->index, NULL);
			    result |= f_close(&logfile);
            	// uart print for testing
            	// !!!! Ensure the timeout (rn 3000) is long enough to transmit a whole log chunk !!!
            	// HAL_UART_Transmit(&huart4, bufferToPrint->buffer, bufferToPrint->index, 3000);
                bufferToPrint->index = 0;
                bufferToPrint->isFull = false;    
        } else {
            noFullBuffMoments++;
        }
		logInfo("log", "%d %d %d %d", droppedMsgs, fullBuffMoments, critErrs, noFullBuffMoments);
    }
}
