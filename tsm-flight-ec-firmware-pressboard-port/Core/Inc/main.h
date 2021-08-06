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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define en_vlv8_Pin GPIO_PIN_2
#define en_vlv8_GPIO_Port GPIOE
#define en_vlv7_Pin GPIO_PIN_3
#define en_vlv7_GPIO_Port GPIOE
#define en_vlv6_Pin GPIO_PIN_4
#define en_vlv6_GPIO_Port GPIOE
#define en_vlv5_Pin GPIO_PIN_5
#define en_vlv5_GPIO_Port GPIOE
#define en_vlv4_Pin GPIO_PIN_6
#define en_vlv4_GPIO_Port GPIOE
#define en_vlv3_Pin GPIO_PIN_13
#define en_vlv3_GPIO_Port GPIOC
#define en_vlv2_Pin GPIO_PIN_14
#define en_vlv2_GPIO_Port GPIOC
#define en_vlv1_Pin GPIO_PIN_15
#define en_vlv1_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOC
#define e_hall1A_Pin GPIO_PIN_0
#define e_hall1A_GPIO_Port GPIOA
#define e_hall1B_Pin GPIO_PIN_1
#define e_hall1B_GPIO_Port GPIOA
#define e_hall1C_Pin GPIO_PIN_2
#define e_hall1C_GPIO_Port GPIOA
#define adc0_cs_Pin GPIO_PIN_3
#define adc0_cs_GPIO_Port GPIOA
#define adc0_eoc_Pin GPIO_PIN_4
#define adc0_eoc_GPIO_Port GPIOA
#define adc0_cnvst_Pin GPIO_PIN_4
#define adc0_cnvst_GPIO_Port GPIOC
#define adc1_eoc_Pin GPIO_PIN_5
#define adc1_eoc_GPIO_Port GPIOC
#define adc1_cs_Pin GPIO_PIN_0
#define adc1_cs_GPIO_Port GPIOB
#define adc1_cnvst_Pin GPIO_PIN_1
#define adc1_cnvst_GPIO_Port GPIOB
#define adc2_cs_Pin GPIO_PIN_2
#define adc2_cs_GPIO_Port GPIOB
#define adc2_eoc_Pin GPIO_PIN_7
#define adc2_eoc_GPIO_Port GPIOE
#define adc2_cnvst_Pin GPIO_PIN_8
#define adc2_cnvst_GPIO_Port GPIOE
#define tc0_cs_Pin GPIO_PIN_9
#define tc0_cs_GPIO_Port GPIOE
#define tc1_cs_Pin GPIO_PIN_10
#define tc1_cs_GPIO_Port GPIOE
#define tc2_cs_Pin GPIO_PIN_11
#define tc2_cs_GPIO_Port GPIOE
#define tc3_cs_Pin GPIO_PIN_15
#define tc3_cs_GPIO_Port GPIOE
#define tc4_cs_Pin GPIO_PIN_10
#define tc4_cs_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_8
#define FLASH_CS_GPIO_Port GPIOD
#define mtr2_in0_Pin GPIO_PIN_9
#define mtr2_in0_GPIO_Port GPIOD
#define mtr2_in1_Pin GPIO_PIN_10
#define mtr2_in1_GPIO_Port GPIOD
#define mtr2_in2_Pin GPIO_PIN_11
#define mtr2_in2_GPIO_Port GPIOD
#define e_hall0A_Pin GPIO_PIN_12
#define e_hall0A_GPIO_Port GPIOD
#define e_hall0B_Pin GPIO_PIN_13
#define e_hall0B_GPIO_Port GPIOD
#define e_hall0C_Pin GPIO_PIN_14
#define e_hall0C_GPIO_Port GPIOD
#define mtr2_pwm0_Pin GPIO_PIN_6
#define mtr2_pwm0_GPIO_Port GPIOC
#define mtr2_pwm1_Pin GPIO_PIN_7
#define mtr2_pwm1_GPIO_Port GPIOC
#define mtr2_pwm2_Pin GPIO_PIN_8
#define mtr2_pwm2_GPIO_Port GPIOC
#define mtr0_inB0_Pin GPIO_PIN_9
#define mtr0_inB0_GPIO_Port GPIOC
#define mtr3_pwm0_Pin GPIO_PIN_8
#define mtr3_pwm0_GPIO_Port GPIOA
#define mtr3_pwm1_Pin GPIO_PIN_9
#define mtr3_pwm1_GPIO_Port GPIOA
#define mtr3_pwm2_Pin GPIO_PIN_10
#define mtr3_pwm2_GPIO_Port GPIOA
#define mtr0_inB1_Pin GPIO_PIN_11
#define mtr0_inB1_GPIO_Port GPIOA
#define GPIO_3_Pin GPIO_PIN_12
#define GPIO_3_GPIO_Port GPIOA
#define mtr0_inA0_Pin GPIO_PIN_10
#define mtr0_inA0_GPIO_Port GPIOC
#define mtr0_inA1_Pin GPIO_PIN_11
#define mtr0_inA1_GPIO_Port GPIOC
#define mtr3_in0_Pin GPIO_PIN_12
#define mtr3_in0_GPIO_Port GPIOC
#define mtr3_in1_Pin GPIO_PIN_0
#define mtr3_in1_GPIO_Port GPIOD
#define mtr3_in2_Pin GPIO_PIN_1
#define mtr3_in2_GPIO_Port GPIOD
#define mtr1_inA0_Pin GPIO_PIN_2
#define mtr1_inA0_GPIO_Port GPIOD
#define mtr1_inA1_Pin GPIO_PIN_3
#define mtr1_inA1_GPIO_Port GPIOD
#define mtr1_inB0_Pin GPIO_PIN_4
#define mtr1_inB0_GPIO_Port GPIOD
#define mtr1_inB1_Pin GPIO_PIN_7
#define mtr1_inB1_GPIO_Port GPIOD
#define mtr1_pwmA_Pin GPIO_PIN_4
#define mtr1_pwmA_GPIO_Port GPIOB
#define mtr1_pwmB_Pin GPIO_PIN_5
#define mtr1_pwmB_GPIO_Port GPIOB
#define mtr0_pwmA_Pin GPIO_PIN_8
#define mtr0_pwmA_GPIO_Port GPIOB
#define mtr0_pwmB_Pin GPIO_PIN_9
#define mtr0_pwmB_GPIO_Port GPIOB
#define GPIO_1_Pin GPIO_PIN_0
#define GPIO_1_GPIO_Port GPIOE
#define en_vlv0_Pin GPIO_PIN_1
#define en_vlv0_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
