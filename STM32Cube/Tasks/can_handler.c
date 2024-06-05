#include "can_handler.h"
#include "flight_phase.h"
#include "trajectory.h"

xQueueHandle busQueue;

void can_handle_rx(const can_msg_t *message, uint32_t timestamp){
	uint16_t msgtype = get_message_type(message);
	BaseType_t isr_thing = pdFALSE;

	if(msgtype == MSG_LEDS_ON)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	}
	else if (msgtype == MSG_LEDS_OFF)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	}
	else if(msgtype == MSG_ACTUATOR_CMD && get_actuator_id(message) == ACTUATOR_INJECTOR_VALVE && get_curr_actuator_state(message) == ACTUATOR_ON)
	{
		xEventGroupSetBits(flightPhaseEventsHandle, INJ_OPEN_BIT); //
	}
	else if (msgtype == MSG_SENSOR_ALTITUDE)
	{
		AltTime result;
		get_altitude_data(message, &result.alt);
		result.time = (float) timestamp;
		xQueueOverwriteFromISR(altQueue, &result, &isr_thing);
		portYIELD_FROM_ISR(isr_thing);
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
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); //write and toggle D3 when we send a CAN message
	}
  }
}

void canHandlerInit(void)
{
	busQueue = xQueueCreate(16, sizeof(can_msg_t));
	can_init_stm(&hfdcan1, can_handle_rx);
}
