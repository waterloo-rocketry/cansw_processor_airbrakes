#include "can_handler.h"


uint32_t LED_state = 0;
xQueueHandle busQueue;

void can_handle_rx(const can_msg_t *message, uint32_t timestamp){
	if(get_message_type(message) == MSG_LEDS_ON)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	}
	else if (get_message_type(message) == MSG_LEDS_OFF)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	}
	return;
}

void canHandlerTask(void *argument)
{
  for(;;)
  {
	can_msg_t tx_msg;
	//Block the thread until we see data in the bus queue or 5 ticks elapse
	if(xQueueReceive(busQueue, &tx_msg, 10) == pdTRUE) //Returns pdTRUE if we got a message, pdFALSE if timed out
	{
		can_send(&tx_msg);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, LED_state); //write and toggle D3 when we send a CAN message
		LED_state = !LED_state;
	}
  }
}

void canHandlerInit(void)
{
	busQueue = xQueueCreate(16, sizeof(can_msg_t));
	can_init_stm(&hfdcan1, can_handle_rx);
}
