/*
 * peripheral_mappings.h
 *
 *  Created on: Jun 20, 2021
 *      Author: natha
 *
 *  Header file to define mappings between peripherals and
 *  their readable names, for programming convenience
 */

#ifndef INC_PERIPHERAL_MAPPINGS_H_
#define INC_PERIPHERAL_MAPPINGS_H_

// SPI bus mappings
#define SPI_TC            hspi1
#define SPI_ADC           hspi2
#define SPI_MTR           hspi3
#define SPI_MEM           hspi4

// Communication mappings
#define COM_UART          huart1
#define CAN_BUS           hcan1

// Timer mappings
#define TIM_10Hz
#define TIM_200Hz
#define TIM_MICROS

#endif /* INC_PERIPHERAL_MAPPINGS_H_ */
