/*
 * fake_logging.h
 *
 *  Created on: Apr 8, 2024
 *      Author: joedo
 */

#ifndef FAKE_LOGGING_H_
#define FAKE_LOGGING_H_

#include "freertos.h"
#include "queue.h"

extern QueueHandle_t logQueue;

typedef struct {
	char data[50];
	uint32_t timestamp;
} logMsg_t;

void fakeLogTask(void *argument);

#endif /* FAKE_LOGGING_H_ */
