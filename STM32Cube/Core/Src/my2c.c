#include <stdbool.h>
#include "my2c.h"

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_i2c.h"

#include "FreeRTOS.h"
#include "semphr.h"

SemaphoreHandle_t I2C4BinarySemaphore;

extern I2C_HandleTypeDef hi2c4;

uint8_t timeout_MS = 200;

/**
 * Register i2c4 callbacks, semaphore
 */
bool MY2C_init(void)
{
    if (HAL_I2C_RegisterCallback(&hi2c4, HAL_I2C_MEM_TX_COMPLETE_CB_ID, &I2C4_MemTxCallback) != HAL_OK)
    {
        return false;
    }

    if (HAL_I2C_RegisterCallback(&hi2c4, HAL_I2C_MEM_RX_COMPLETE_CB_ID, &I2C4_MemRxCallback) != HAL_OK)
    {
        return false;
    }

    if (HAL_I2C_RegisterCallback(&hi2c4, HAL_I2C_ERROR_CB_ID, I2C4_ErrorCallback) != HAL_OK)
    {
        return false;
    }

	I2C4BinarySemaphore = xSemaphoreCreateBinary();

	if (I2C4BinarySemaphore)
	{
		return true;
	}

	return false;
}

/**
 * Internal helper read function, read N bytes using interrupt handling
 */
HAL_StatusTypeDef MY2C_readNByteRegister_IT(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len)
{
	HAL_StatusTypeDef status = HAL_OK;

    // request async write, returning HAL_OK if it successfully started the write
	status = HAL_I2C_Mem_Read_IT(&hi2c4, address << 1, reg, 1, data, len);

    if (status != HAL_OK)
    {
    	return status;
    }

    // semaphore is given in the I2C4 interrupt when the write succeeded
	if (xSemaphoreTake(I2C4BinarySemaphore, pdMS_TO_TICKS(timeout_MS)) != pdTRUE)
	{
		// semaphore timed out; handle error?
	}

	return status;
}


/**
 * Internal helper write function, write N bytes using interrupt handling
 */
HAL_StatusTypeDef MY2C_writeNByteRegister_IT(uint8_t address, uint8_t reg, uint8_t data, uint8_t len)
{
	HAL_StatusTypeDef status = HAL_OK;
    
    // request async write, returning HAL_OK if it successfully started the write
	status = HAL_I2C_Mem_Write_IT(&hi2c4, address << 1, reg, 1, &data, len);

    if (status != HAL_OK)
    {
    	return status;
    }

    // semaphore is given in the I2C4 interrupt when the write succeeded
	if (xSemaphoreTake(I2C4BinarySemaphore, pdMS_TO_TICKS(timeout_MS)) != pdTRUE)
	{
		// semaphore timed out; handle error?
	}

	return status;
}

/**
 * Read 1 byte register synchronously
 */
uint8_t MY2C_read1ByteRegister(uint8_t address, uint8_t reg)
{
	uint8_t data;
	MY2C_readNByteRegister_IT(address, reg, &data, 1);
	return data;
}

/**
 * Write 1 byte register synchronously
 */
HAL_StatusTypeDef MY2C_write1ByteRegister(uint8_t address, uint8_t reg, uint8_t data)
{
	return MY2C_writeNByteRegister_IT(address, reg, data, 1);
}

// Called from I2C4 ISR after a successful read 
void I2C4_MemRxCallback(I2C_HandleTypeDef* hi2c4)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; 

    if (xSemaphoreGiveFromISR(I2C4BinarySemaphore, &xHigherPriorityTaskWoken) != pdTRUE)
    {
        // semaphore timed out; handle this?
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Called from I2C4 ISR after a successful write 
void I2C4_MemTxCallback(I2C_HandleTypeDef* hi2c4)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; 

    if (xSemaphoreGiveFromISR(I2C4BinarySemaphore, &xHigherPriorityTaskWoken) != pdTRUE)
    {
        // semaphore timed out; handle this?
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Called from I2C4 ISR after I2C error
void I2C4_ErrorCallback(I2C_HandleTypeDef* hi2c4)
{
	// handle i2c read/write give up error
}

