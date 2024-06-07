#include "can_handler.h"
#include "flight_phase.h"
#include "trajectory.h"
#include "millis.h"

xQueueHandle busQueue;

void can_handle_rx(const can_msg_t *message, uint32_t timestamp){
	//The timestamp parameter passed to the handler is some internal FDCAN thing that I don't know how to convert to a sensible value
	//Just use millis_() for now
	uint16_t msgtype = get_message_type(message);
	BaseType_t xHigherPriorityTaskWoken, result = pdFALSE;

	if(msgtype == MSG_LEDS_ON)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	}
	else if (msgtype == MSG_LEDS_OFF)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	}
	else if(msgtype == MSG_ACTUATOR_CMD && get_actuator_id(message) == ACTUATOR_INJECTOR_VALVE && get_req_actuator_state(message) == ACTUATOR_ON)
	{
		result = xEventGroupSetBitsFromISR(flightPhaseEventsHandle, INJ_OPEN_BIT, &xHigherPriorityTaskWoken);
	}
	else if (msgtype == MSG_SENSOR_ALTITUDE)
	{
		AltTime data;
		get_altitude_data(message, &data.alt);
		data.time = millis_();
		result = xQueueOverwriteFromISR(altQueue, &data, &xHigherPriorityTaskWoken);
	}

	/*this will potentially yield from the CAN callback early but that is okay so long as the
	CAN ISR imlementation doesn't do any cleanup after returning from the callback (it currently doesn't) */
	if(result != pdFALSE)  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

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
