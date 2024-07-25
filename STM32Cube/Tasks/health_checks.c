/*
 * health_checks.c
 *
 *  Created on: May 1, 2024
 *      Author: joedo
 */

#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"

#include "printf.h"
#include "millis.h"

#include "health_checks.h"
#include "log.h"
#include "can_handler.h"
#include "otits.h"

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
        res.info = "raw adc > 65536";
    	return res;
    } else if (adc1_current_mA > 500) {
        res.outcome = TEST_OUTCOME_DATA_ERR;
        res.info = "I > 500mA";
    	return res;
    } else if (adc1_current_mA < 100) {
        res.outcome = TEST_OUTCOME_DATA_ERR;
        res.info = "I < 100mA";
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
  TickType_t lastWakeTime = xTaskGetTickCount();
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
    build_board_stat_msg(millis_(), E_5V_OVER_CURRENT, current_data, 2, &msg);
    xQueueSend(busQueue, &msg, 10);
    } else {
    // E nominal
    can_msg_t msg;
    build_board_stat_msg(0, E_NOMINAL, NULL, 0, &msg);
    xQueueSend(busQueue, &msg, 10);

    // Current Draw Message
    build_analog_data_msg(0, SENSOR_BATT_CURR, adc1_current_mA, &msg);
    xQueueSend(busQueue, &msg, 10);


    logError("health", "over current %dmA", adc1_current_mA);
    }

    vTaskDelayUntil(&lastWakeTime, 1000);
  }
  /* USER CODE END healthCheckTask */
}
