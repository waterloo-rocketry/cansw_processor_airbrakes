/*
 * health_checks.c
 *
 *  Created on: May 1, 2024
 *      Author: joedo
 */

#include "health_checks.h"
#include "log.h"
#include "can_handler.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart4;

void healthCheckTask(void *argument)
{
/*
  Note: Currently ADC1 has channels 10 and 11, polling both at a time
  is very possible, through a variety of methods (DMA), however
  for testing purposes until the ADC dudes can go in person to do it,
  I recommend just using CubeMX. If you go to rank under ADC1 in CubeMX,
  you will see Channel 10, once you are done testing channel 10, switch it
  to channel 11 (can also be done in the CubeMX generated code on line ~420
  in the config channel block), then the other channel can be tested.
  */

  uint8_t adc_strval[20]; //tx string buffer
  uint32_t adc1_val;

  // Calibrate ADC
  HAL_ADC_Stop(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED );
  HAL_ADC_Start(&hadc1);

  /* Infinite loop */
  for (;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc1_val = HAL_ADC_GetValue(&hadc1);
    uint16_t current = CURR_5V_mA(adc1_val);

    // Sending ADC val over uart
    //int adc_txlength = sprintf((char*) adc_strval, "%u mV\r\n", (uint16_t) (ADC1_VOLTAGE(adc1_val) * 1000));
    //HAL_UART_Transmit(&huart4, (uint8_t*) adc_strval, adc_txlength, 10);

    //Log current with logging api
    logDebug(SOURCE_HEALTH, "5V Current: %u mA", current);

    if(current > MAX_CURR_5V_mA)
    {
    	can_msg_t msg;
    	uint8_t current_data[2];
    	current_data[0] = current >> 8 && 0xFF;
    	current_data[1] = current && 0xFF;
    	build_board_stat_msg(0, E_5V_OVER_CURRENT, current_data, 2, &msg);
    	xQueueSend(busQueue, &msg, 10);
    }


    vTaskDelay(1000);
  }
  /* USER CODE END healthCheckTask */
}
