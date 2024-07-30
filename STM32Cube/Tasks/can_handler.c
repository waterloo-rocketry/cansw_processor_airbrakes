#include "can_handler.h"
#include "flight_phase.h"
#include "trajectory.h"
#include "controller.h"
#include "state_estimation.h"
#include "millis.h"
#include "log.h"

xQueueHandle busQueue;

void can_handle_rx(const can_msg_t *message, uint32_t timestamp) {
	//The timestamp parameter passed to the handler is some internal FDCAN thing that I don't know how to convert to a sensible value
	//Just use millis_() for now
	uint16_t msgType = get_message_type(message);
	BaseType_t xHigherPriorityTaskWoken, result = pdFALSE;

	switch(msgType) {
	case MSG_LEDS_ON:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
        break;

	case MSG_LEDS_OFF:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
        break;

	case MSG_ACTUATOR_CMD:
	    if (get_actuator_id(message) == ACTUATOR_INJECTOR_VALVE && get_req_actuator_state(message) == ACTUATOR_ON) {
	        result = xEventGroupSetBitsFromISR(flightPhaseEventsHandle, INJ_OPEN_BIT, &xHigherPriorityTaskWoken);
	    }
	    break;

	case MSG_SENSOR_ALTITUDE:
	    AltTime data;
	    get_altitude_data(message, &data.alt); //altimeter raw data comes in in feet, we use m like god intended
	    data.alt = data.alt / FT_TO_M;
	    data.time = millis_();
	    result = xQueueOverwriteFromISR(altQueue, &data, &xHigherPriorityTaskWoken);
	    break;
	case MSG_STATE_EST_CALIB:
		uint8_t flag; //sent to processor low
		uint16_t apogee;
		if(get_state_est_calibration_msg(message, &flag, &apogee) && !flag) //left side is guaranteed to evaluate before right side
		{
			result |= xQueueSendFromISR(targetQueue, &apogee, &xHigherPriorityTaskWoken); //send new target apogee to controller
			result |= xEventGroupSetBitsFromISR(calibrationEventHandle, RESET_FILTER_FLAG, &xHigherPriorityTaskWoken); //tell state estimation to reset filter algorithm
		}
		break;

	default:
		break;
	}

	/*this will potentially yield from the CAN callback early but that is okay so long as the
	CAN ISR imlementation doesn't do any cleanup after returning from the callback (it currently doesn't) */
	if(result != pdFALSE)  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	return;
}

void canHandlerTask(void *argument) {
    for(;;) {
        can_msg_t tx_msg;
        //Block the thread until we see data in the bus queue or 1 sec elapses
        if(xQueueReceive(busQueue, &tx_msg, 1000) == pdTRUE) { //Returns pdTRUE if we got a message, pdFALSE if timed out
            if(can_send(&tx_msg) == false)
            {
            	logError("CAN", "CAN send failed!");
            }
            vTaskDelay(1);
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10); //write and toggle D3 when we send a CAN message
        }
    }
}

bool canHandlerInit(void) {
	busQueue = xQueueCreate(16, sizeof(can_msg_t));
	if (busQueue == NULL) return false;
	if (!can_init_stm(&hfdcan1, can_handle_rx)) return false;
	return true;
}
