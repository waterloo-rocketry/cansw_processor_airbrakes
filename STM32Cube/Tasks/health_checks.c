/*
 * health_checks.c
 *
 *  Created on: May 1, 2024
 *      Author: joedo
 */

#include "health_checks.h"
#include "log.h"
#include "can_handler.h"
#include "otits.h"
#include "printf.h"

extern ADC_HandleTypeDef hadc1;

static Otits_Result_t test_ADCRead() {
    Otits_Result_t res;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t adc1_val = HAL_ADC_GetValue(&hadc1);
    uint16_t adc1_voltage_mV = ADC1_VOLTAGE_V(adc1_val)*1000;
    uint16_t adc1_current_mA = ADC1_CURR_mA(adc1_voltage_mV);

    if (adc1_val > 65536 ) {
        res.outcome = TEST_OUTCOME_DATA_ERR;
        res.info = "raw adc out of range";
    	return res;
    } else if (adc1_current_mA > 500) {
        res.outcome = TEST_OUTCOME_DATA_ERR;
        res.info = "I > 500mA";
    	return res;
    }
    res.outcome = TEST_OUTCOME_PASSED;
    res.info = "";
	return res;
}

bool healthCheckInit() {
    HAL_StatusTypeDef sts = HAL_OK;
    // Calibrate ADC
    sts |= HAL_ADC_Stop(&hadc1);
    sts |= HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED );
    sts |= HAL_ADC_Start(&hadc1);
    if (sts != HAL_OK) return false;

    if (!otitsRegister(test_ADCRead, TEST_SOURCE_HEALTH, "health")) return false;

    return true;
}

void healthCheckTask(void *argument)
{
  /* Infinite loop */
  for (;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t adc1_val = HAL_ADC_GetValue(&hadc1);
    uint16_t adc1_voltage_mV = ADC1_VOLTAGE_V(adc1_val)*1000;
    uint16_t adc1_current_mA = ADC1_CURR_mA(adc1_voltage_mV);

    //Transmitting voltage
    //printf_( "%u mV\r\n", (uint16_t) (adc1_voltage_mV));

    //Transmitting current
    //printf_("%u mA\r\n", (uint16_t) (adc1_current_mA) );;

    //Checking for over current
    if(adc1_current_mA > MAX_CURR_5V_mA)
    {
    can_msg_t msg;
    uint8_t current_data[2];
    current_data[0] = adc1_current_mA >> 8 & 0xFF;
    current_data[1] = adc1_current_mA & 0xFF;
    build_board_stat_msg(0, E_5V_OVER_CURRENT, current_data, 2, &msg);
    xQueueSend(busQueue, &msg, 10);
    }

    vTaskDelay(100);
  }
  /* USER CODE END healthCheckTask */
}
