/*
 * constants.h
 *
 *  Created on: Jun 28, 2021
 *      Author: natha
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#include "stm32f4xx.h"  // _HandleTypeDef definitions
#include "main.h"  // gpio pin mappings

// Extern declarations from main.c
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;


// Addresses
#define FLIGHT_COMP_ADDR  (1)
#define FLIGHT_EC_ADDR    (2)
#define SERVER_ADDR       (7)

// SPI bus mappings
#define SPI_TC            hspi1
#define SPI_ADC           hspi2
#define SPI_MOTOR         hspi3
#define SPI_MEM           hspi4

// Communication mappings
#define COM_UART          huart1
#define CAN_BUS           hcan1

// Timer mappings
#define TIM_5MS           htim13
#define TIM_50MS          htim10
#define TIM_100MS         htim11
#define TIM_MICROS        htim5

// LED mappings
#define LED_TELEM_PORT              LED_0_GPIO_Port
#define LED_TELEM_PIN               LED_0_Pin
#define LED_FLASH_LOGGING_PORT      LED_1_GPIO_Port
#define LED_FLASH_LOGGING_PIN       LED_1_GPIO_Pin

// Microseconds since board reset
#define SYS_MICROS        ((uint32_t)(__HAL_TIM_GET_COUNTER(&TIM_MICROS)))
#define SYS_MILLIS        ((uint32_t)(SYS_MICROS/1000))  // Not really needed

/*
 * Sensor channel mappings
 * TODO: these mappings are all wrong
 */
// Pressure transducers and potentiometers
#define CHAMBER_PRES_CH                (0)
#define REGEN_INJ_PRES_CH              (1)  // TODO: 2 PTs are named regen?
#define REGEN_FUEL_LINE_PRES_CH        (2)
#define LOX_DOME_PRES_CH               (3)
#define LOX_TANK_A_PRES_CH             (4)
#define LOX_TANK_B_PRES_CH             (5)
#define LOX_TANK_C_PRES_CH             (6)
#define LOX_TANK_DIF_PRES_CH           (7)
#define FUEL_TANK_A_PRES_CH            (8)
#define FUEL_TANK_B_PRES_CH            (9)
#define FUEL_TANK_C_PRES_CH           (10)
#define FUEL_TANK_DIF_PRES_CH         (11)
#define COPV_A_PRES_CH                (12)
#define COPV_B_PRES_CH                (13)
#define COPV_C_PRES_CH                (14)
#define ACTUATE_PRES_CH               (15)  // todo: wtf is this?
#define LOX_PNEU_CAV_PRES_CH          (16)
#define FUEL_PNEU_CAV_PRES_CH         (17)
#define CHAMBER_2_PRES_CH             (18)
#define NOZZLE_FILM_PRES_CH           (19)
#define POT_0_CH                      (20)  // TODO: A/B or 0/1 or 1/2?
#define POT_1_CH                      (21)

// Thermocouples
// TODO: fix these
#define FIN_TEMP_CH                    (0)
#define AV_BAY_TEMP_CH                 (1)
#define RACEWAY_A_TEMP_CH              (2)
#define RACEWAY_B_TEMP_CH              (3)
#define CHAMBER_TEMP_CH                (4)
#define REGEN_TEMP_CH                  (5)
#define NOZZLE_TEMP_CH                 (6)
#define COPV_TEMP_CH                   (7)
#define LOX_ULLAGE_TEMP_CH             (8)
#define LOX_TEMP_CH                    (9)
#define FUEL_ULLAGE_TEMP_CH           (10)
#define FUEL_TEMP_CH                  (11)

/**
 * Misc definitions of system parameters
 */
#define NUM_TANKS                      (2)
#define NUM_VALVES                    (14)
#define NUM_TCS                       (12)
#define NUM_PTS                       (20)
#define NUM_POTS                       (2)

#define LOX_TANK                       (0)
#define FUEL_TANK                      (1)

#endif /* INC_CONSTANTS_H_ */
