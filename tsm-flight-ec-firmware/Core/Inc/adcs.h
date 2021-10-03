/*
 * adc.h
 *
 *  Created on: Oct 2, 2021
 *      Author: natha
 */

#ifndef INC_ADCS_H_
#define INC_ADCS_H_

#include "stm32f4xx_hal.h"

/**
 * Call this function once in the initialization
 * Sets pin configurations for the ADCs
 * See schematic for pin configuration
 */
void init_adcs();

/**
 * Call this function in a periodic loop to read analog sensors
 *
 * hspi is a pointer to a SPI configuration struct of the
 * bus that the ADCs run on
 */
void read_adcs(SPI_HandleTypeDef * hspi);

#endif /* INC_ADCS_H_ */
