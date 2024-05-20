#include "can_handler.h"
#include "flight_phase.h"


uint32_t LED_state = 0;
xQueueHandle busQueue;


void can_handle_rx(const can_msg_t *message, uint32_t timestamp){
	uint16_t msg_type = get_message_type(message);
	logDebug(SOURCE_CAN_RX, "CAN MSG: sid: %u,", msg_type); //TODO make more verbose

	switch(msg_type)
	{
	case MSG_ACTUATOR_CMD:
		if(get_actuator_id(message) == ACTUATOR_INJECTOR_VALVE && get_req_actuator_state(message) == ACTUATOR_ON)
		{
			xEventGroupSetBits(flightPhaseEventsHandle, INJ_OPEN_BIT); //these never get cleared so no need to check return val
		}
		//TODO detect drogue firing
	case MSG_LEDS_ON:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		break;
	case MSG_LEDS_OFF:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		break;
	default:
		//we don't care about this message
		break;
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
