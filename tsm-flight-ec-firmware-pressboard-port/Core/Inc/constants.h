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
//extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

//extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;


// Addresses
#define GSE_CONTROLLER_ADDR  (0)
#define FLIGHT_COMP_ADDR     (1)
#define FLIGHT_EC_ADDR       (3)
#define SERVER_ADDR          (7)

// SPI bus mappings
#define SPI_TC            hspi4
#define SPI_ADC           hspi1
//#define SPI_MOTOR         hspi3
#define SPI_MEM           hspi2

// Communication mappings
#define COM_UART          huart2
//#define CAN_BUS           hcan1

// Timer mappings
/*
#define TIM_5MS           htim13
#define TIM_50MS          htim10
#define TIM_100MS         htim11
#define TIM_MICROS        htim5
*/
#define TIM_5MS           htim7
#define TIM_50MS          htim10
#define TIM_100MS         htim11
#define TIM_MICROS        htim5
#define TIM_MTR0_STEP

// LED mappings
/*
#define LED_TELEM_PORT              (LED_0_GPIO_Port)
#define LED_TELEM_PIN               (LED_0_Pin)
#define LED_FLASH_LOGGING_PORT      (LED_1_GPIO_Port)
#define LED_FLASH_LOGGING_PIN       (LED_1_Pin)
*/
#define LED_TELEM_PORT              (LED0_GPIO_Port)
#define LED_TELEM_PIN               (LED0_Pin)
#define LED_FLASH_LOGGING_PORT      (LED1_GPIO_Port)
#define LED_FLASH_LOGGING_PIN       (LED1_Pin)

// Microseconds since board reset
#define SYS_MICROS        ((uint32_t)(__HAL_TIM_GET_COUNTER(&TIM_MICROS)))
#define SYS_MILLIS        ((uint32_t)(SYS_MICROS/1000))  // Not really needed

/**
 * Valve channel mappings
 * TODO: fix these
 */
#define LOX_CONTROL_VALVE_CH          (0)
#define LOX_MPV_VALVE_CH              (1)
#define LOX_TANK_VENT_VALVE_CH        (2)
#define FUEL_CONTROL_VALVE_CH         (3)
#define FUEL_MPV_PRESS_VALVE_CH       (4)
#define FUEL_MPV_VENT_VALVE_CH        (5)
#define IGNITOR_CH                    (6)
#define PURGE_VALVE_CH                (7)
#define NOZZLE_FILM_COOLING_VALVE_CH  (8)

#define GSE_FUEL_TANK_VENT_VALVE_CH   (9)
/*
 * Sensor channel mappings
 * TODO: these mappings are all wrong
 */
// Pressure transducers and potentiometers
#define LOX_TANK_PRES_CH               (0)
#define CHAMBER_PRES_CH                (1)
#define FUEL_TANK_PRES_CH              (2)
#define LOX_INJ_MANIFOLD_PRES_CH       (3)
#define COPV_PRES_CH                   (4)
#define FUEL_INJ_MANIFOLD_PRES_CH      (5)

#define POT_0_CH                       (0)
#define POT_1_CH                       (1)

// Thermocouples
// TODO: fix these
// TODO
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
#define NUM_VALVES                     (9)
#define NUM_TCS                        (5)
#define NUM_PTS                        (6)
#define NUM_POTS                       (2)
#define NUM_MOTORS                     (2)

#define LOX_TANK_NUM                   (0)
#define FUEL_TANK_NUM                  (1)

#endif /* INC_CONSTANTS_H_ */
