#ifndef __CAN_HANDLER
#define __CAN_HANDLER

#ifdef __cplusplus
extern "C" {
#endif

#include "canlib.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern xQueueHandle busQueue;


void can_handle_rx(const can_msg_t *message, uint32_t timestamp);
void canHandlerTask(void *argument);
void canHandlerInit(void);


#ifdef __cplusplus
}
#endif


#endif
