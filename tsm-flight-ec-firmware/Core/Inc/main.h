/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC2_EOC_Pin GPIO_PIN_3
#define ADC2_EOC_GPIO_Port GPIOE
#define ADC2_CS_Pin GPIO_PIN_4
#define ADC2_CS_GPIO_Port GPIOE
#define FLASH_CS_Pin GPIO_PIN_13
#define FLASH_CS_GPIO_Port GPIOC
#define EEPROM_CS_Pin GPIO_PIN_14
#define EEPROM_CS_GPIO_Port GPIOC
#define LED_0_Pin GPIO_PIN_0
#define LED_0_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOC
#define ADC3_EOC_Pin GPIO_PIN_1
#define ADC3_EOC_GPIO_Port GPIOA
#define ADC3_CS_Pin GPIO_PIN_2
#define ADC3_CS_GPIO_Port GPIOA
#define TC_MUX_EN_Pin GPIO_PIN_4
#define TC_MUX_EN_GPIO_Port GPIOC
#define TC_MUX_A3_Pin GPIO_PIN_5
#define TC_MUX_A3_GPIO_Port GPIOC
#define TC_MUX_A2_Pin GPIO_PIN_0
#define TC_MUX_A2_GPIO_Port GPIOB
#define TC_MUX_A1_Pin GPIO_PIN_1
#define TC_MUX_A1_GPIO_Port GPIOB
#define TC_MUX_A0_Pin GPIO_PIN_2
#define TC_MUX_A0_GPIO_Port GPIOB
#define en_vlv0_Pin GPIO_PIN_13
#define en_vlv0_GPIO_Port GPIOE
#define en_vlv1_Pin GPIO_PIN_14
#define en_vlv1_GPIO_Port GPIOE
#define en_vlv2_Pin GPIO_PIN_15
#define en_vlv2_GPIO_Port GPIOE
#define en_vlv3_Pin GPIO_PIN_10
#define en_vlv3_GPIO_Port GPIOB
#define en_vlv4_Pin GPIO_PIN_12
#define en_vlv4_GPIO_Port GPIOB
#define en_vlv5_Pin GPIO_PIN_8
#define en_vlv5_GPIO_Port GPIOD
#define en_vlv6_Pin GPIO_PIN_9
#define en_vlv6_GPIO_Port GPIOD
#define en_vlv7_Pin GPIO_PIN_10
#define en_vlv7_GPIO_Port GPIOD
#define en_vlv8_Pin GPIO_PIN_11
#define en_vlv8_GPIO_Port GPIOD
#define en_vlv9_Pin GPIO_PIN_12
#define en_vlv9_GPIO_Port GPIOD
#define ADC0_EOC_Pin GPIO_PIN_13
#define ADC0_EOC_GPIO_Port GPIOD
#define ADC0_CS_Pin GPIO_PIN_14
#define ADC0_CS_GPIO_Port GPIOD
#define ADC1_EOC_Pin GPIO_PIN_15
#define ADC1_EOC_GPIO_Port GPIOD
#define ADC1_CS_Pin GPIO_PIN_6
#define ADC1_CS_GPIO_Port GPIOC
#define en_vlv10_Pin GPIO_PIN_7
#define en_vlv10_GPIO_Port GPIOC
#define en_vlv11_Pin GPIO_PIN_8
#define en_vlv11_GPIO_Port GPIOC
#define en_vlv12_Pin GPIO_PIN_9
#define en_vlv12_GPIO_Port GPIOC
#define en_vlv13_Pin GPIO_PIN_8
#define en_vlv13_GPIO_Port GPIOA
#define MTR0_BUSY_Pin GPIO_PIN_9
#define MTR0_BUSY_GPIO_Port GPIOA
#define MTR0_CS_Pin GPIO_PIN_10
#define MTR0_CS_GPIO_Port GPIOA
#define MTR1_BUSY_Pin GPIO_PIN_11
#define MTR1_BUSY_GPIO_Port GPIOA
#define MTR1_CS_Pin GPIO_PIN_12
#define MTR1_CS_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
