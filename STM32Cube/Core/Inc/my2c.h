#ifndef MY2C_H
#define	MY2C_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx_hal.h"

bool MY2C_init(void);

HAL_StatusTypeDef MY2C_write1ByteRegister(uint8_t address, uint8_t reg, uint8_t data);

uint8_t MY2C_read1ByteRegister(uint8_t address, uint8_t reg);

#endif	/* MY2C_H */
