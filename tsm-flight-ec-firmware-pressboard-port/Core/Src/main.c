/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "constants.h"
#include "autosequence.h"
//#include "sensors.h"
#include "globals.h"  // included for STATE
#include "serial_data.h"
#include "tank_pressure_control.h"
#include "nonvolatile_memory.h"
#include "hardware.h"

#include <string.h>

//#include "L6470.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

// Timer interrupt flags
volatile uint8_t periodic_flag_5ms;
volatile uint8_t periodic_flag_50ms;
volatile uint8_t periodic_flag_100ms;

// Serial data
extern W25N01GV_Flash flash;
extern uint8_t telem_disabled;
//extern DmaBufferInfo buffer_info;

// Autosequence control info and timings
extern Autosequence_Info autosequence;

// DMA
#define DMA_RX_BUFFER_SIZE          2048
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define NUM_BUFFER_PACKETS          10
#define CIRCULAR_TELEM_BUFFER_SZ    PONG_MAX_PACKET_SIZE*NUM_BUFFER_PACKETS
uint8_t circular_telem_buffer[CIRCULAR_TELEM_BUFFER_SZ];
uint8_t temp_telem_buffer[PONG_MAX_PACKET_SIZE] = { 0 };
uint8_t daisy_telem_buffer[CIRCULAR_TELEM_BUFFER_SZ] = { 0 };
volatile int16_t curr_circular_buffer_pos               = 0;
volatile int16_t curr_telem_start[NUM_BUFFER_PACKETS]   = { 0 };
volatile int16_t curr_telem_len[NUM_BUFFER_PACKETS]     = { 0 };
volatile int16_t last_telem_packet_pos                  = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM13_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Timer interrupt flag handling
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim == &TIM_5MS) {
		periodic_flag_5ms = 1;
	}
	else if (htim == &TIM_50MS) {
		periodic_flag_50ms = 1;
	}
	else if (htim == &TIM_100MS) {
		periodic_flag_100ms = 1;
	}
	else if (htim == &TIM_MTR0_STEP) {
		handleMotorStepping(0);
	}
	else if (htim == &TIM_MTR1_STEP) {
		handleMotorStepping(1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t RxRollover  = 0;
    static uint16_t RxBfrPos   = 0;
    // UART Rx Complete Callback;
    // Rx Complete is called by: DMA (automatically), if it rolls over
    // and when an IDLE Interrupt occurs
    // DMA Interrupt allays occurs BEFORE the idle interrupt can be fired because
    // idle detection needs at least one UART clock to detect the bus is idle. So
    // in the case, that the transmission length is one full buffer length
    // and the start buffer pointer is at 0, it will be also 0 at the end of the
    // transmission. In this case the DMA rollover will increment the RxRollover
    // variable first and len will not be zero.
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {                    // Check if it is an "Idle Interrupt"
        __HAL_UART_CLEAR_IDLEFLAG(huart);                             // clear the interrupt

        uint16_t start = RxBfrPos;                                        // Rx bytes start position (=last buffer position)
        RxBfrPos = DMA_RX_BUFFER_SIZE - (uint16_t)huart->hdmarx->Instance->NDTR;// determine actual buffer position
        uint16_t len = DMA_RX_BUFFER_SIZE;                                // init len with max. size

        if(RxRollover < 2)  {
            if(RxRollover) {                                                        // rolled over once
                if(RxBfrPos <= start) {
                    len = RxBfrPos + DMA_RX_BUFFER_SIZE - start;  // no bytes overwritten
                } else {
                    len = DMA_RX_BUFFER_SIZE + 1;                 // bytes overwritten error
                }
            } else {
                len = RxBfrPos - start;                           // no bytes overwritten
            }
        } else {
            len = DMA_RX_BUFFER_SIZE + 2;                         // dual rollover error
        }

        if(len && (len <= DMA_RX_BUFFER_SIZE)) {
            uint16_t bytes_in_first_part = len;
            uint16_t bytes_in_second_part = 0;
            if (RxBfrPos < start) { // if data loops in buffer
                bytes_in_first_part = DMA_RX_BUFFER_SIZE - start;
                bytes_in_second_part= len - bytes_in_first_part;
            }

            // handle telem for yourself immediately
            memcpy(temp_telem_buffer, DMA_RX_Buffer+start, bytes_in_first_part);
            memcpy(temp_telem_buffer+bytes_in_first_part, DMA_RX_Buffer, bytes_in_second_part);
            uint8_t cmd_status = receive_data(huart, temp_telem_buffer, len);

            // handle telem for others in buffer
            /*
            if (cmd_status == CLB_RECEIVE_DAISY_TELEM) {
                curr_telem_start[last_telem_packet_pos]= curr_circular_buffer_pos;
                copyDataToBuffer(temp_telem_buffer, len);
                curr_telem_len[last_telem_packet_pos]  = len;
                last_telem_packet_pos = (last_telem_packet_pos + 1) % NUM_BUFFER_PACKETS;
            }
            */
        } else {
            // buffer overflow error:
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			init_board(OWN_BOARD_ADDR); //Fixes an issue with CLB_board_addr changing
			last_telem_packet_pos = 0;
			HAL_UART_Receive_DMA(&COM_UART, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE); // dma buffer overflowed
        }

        RxRollover = 0;                                                    // reset the Rollover variable
    } else {
        // no idle flag? --> DMA rollover occurred
        RxRollover++;       // increment Rollover Counter
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Initialize everything

  /* Initialize HAL stuff */
  // Timers
  HAL_TIM_Base_Start(&TIM_MICROS);
  HAL_TIM_Base_Start_IT(&TIM_5MS);
  HAL_TIM_Base_Start_IT(&TIM_50MS);
  HAL_TIM_Base_Start_IT(&TIM_100MS);
  HAL_TIM_Base_Start(&TIM_MTR0_PWM);
  HAL_TIM_Base_Start(&TIM_MTR1_PWM);
  HAL_TIM_Base_Start_IT(&TIM_MTR0_STEP);
  HAL_TIM_Base_Start_IT(&TIM_MTR1_STEP);

  // UART DMA
  __HAL_UART_ENABLE_IT(&COM_UART, UART_IT_IDLE);   // enable idle line interrupt
  HAL_UART_Receive_DMA(&COM_UART, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);

  // Read variables from flash: this must be called very early in initialization!
  init_flash(&flash, &SPI_MEM, FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  read_nonvolatile_variables();

  // Initializations
  init_board(OWN_BOARD_ADDR);  // Comms
  init_autosequence_constants();  // Hardcoded values
  init_autosequence_control_variables();
  init_tank_pressure_control_configuration();  // PID
  init_hardware();  // Press board sensors, etc

  // Packet values
  telem_rate = 1000/(TIM_100MS.Init.Period+1);
  adc_rate = 1000/(TIM_5MS.Init.Period+1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Handle autosequence first in every loop
	  // most important, time sensitive operation
	  execute_autosequence();

	  if (periodic_flag_50ms) {
		  periodic_flag_50ms = 0;

		  // Active tank pressure PID control
		  // tank enable flags get set during the autosequence
		  if (STATE == Hotfire) {
			  if (autosequence.hotfire_lox_tank_enable_PID_control) {
				  tank_PID_pressure_control(&tanks[LOX_TANK_NUM]);
			  }
			  if (autosequence.hotfire_fuel_tank_enable_PID_control) {
				  tank_PID_pressure_control(&tanks[FUEL_TANK_NUM]);
			  }
		  }
	  }

	  if (periodic_flag_5ms) {
		  periodic_flag_5ms = 0;

		  // sample adcs and thermocouples
		  readAdcs(&SPI_ADC, adc_pins, adc_counts);
		  readThermocouples(&SPI_TC, tc_pins, 5);
		  updatePeripherals(adc_counts);

		  update_serial_data_vars();

		  // handle redundant sensor voting algorithms
		  //resolve_redundant_sensors();

		  // Autopress bang bang
		  if (STATE == AutoPress) {
			  tank_autopress_bang_bang(&tanks[LOX_TANK_NUM]);
			  tank_autopress_bang_bang(&tanks[FUEL_TANK_NUM]);
		  }

		  // Initial motor position
		  if (STATE == Startup || STATE == Ignition) {
			  autosequence.T_state = get_ellapsed_time_in_autosequence_state_ms();

			  // Initial motor position is arbitrarily put in the 5ms loop
			  if (STATE == Ignition || (STATE == Startup && autosequence.T_state
					  >= autosequence.startup_motor_start_delay_ms)) {
				  // Allow manual transition to Ignition
				  autosequence.startup_init_motor_pos_complete = 1;

				  // Set motors to initial position
				  tank_startup_init_motor_position(&tanks[LOX_TANK_NUM]);
				  tank_startup_init_motor_position(&tanks[FUEL_TANK_NUM]);
			  }
		  }

		  // Active tank pressure control valve bang bang
		  // tank enable flags get set during the autosequence
		  if (STATE == Hotfire) {
			  if (autosequence.hotfire_lox_tank_enable_PID_control) {
				  tank_check_control_valve_threshold(&tanks[LOX_TANK_NUM]);
			  }
			  if (autosequence.hotfire_fuel_tank_enable_PID_control) {
				  tank_check_control_valve_threshold(&tanks[FUEL_TANK_NUM]);
			  }
		  }

		  // log flash data
		  if (LOGGING_ACTIVE) {
			  save_flash_packet();
		  }

		  // Ignitor break detection
		  if (STATE == Ignition && autosequence.T_state
				  >= autosequence.ignition_ignitor_on_delay_ms) {
			  update_ignitor_break_detector();
		  }

		  // Combustion failure detection
		  if (STATE == Hotfire && autosequence.T_state
				  >= autosequence.hotfire_chamber_pres_lower_bound_abort_start_time_ms) {
			  update_combustion_failure_detector();
		  }

	  }



	  // Check periodic interrupt flags and call appropriate functions if needed
	  if (periodic_flag_100ms) {
		  periodic_flag_100ms = 0;

		  if (!telem_disabled) {
			  send_telem_packet(SERVER_ADDR);
			  HAL_GPIO_TogglePin(LED_TELEM_PORT, LED_TELEM_PIN);
		  }
	  }

	  // Refresh watchdog timer to keep the board running
	  HAL_IWDG_Refresh(&hiwdg);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 624;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 44999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 44999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 44;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 44;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 44999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 44999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 49;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 44999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 99;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 44;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, en_vlv8_Pin|en_vlv7_Pin|en_vlv6_Pin|en_vlv5_Pin
                          |en_vlv4_Pin|GPIO_1_Pin|en_vlv0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, en_vlv3_Pin|en_vlv2_Pin|en_vlv1_Pin|LED0_Pin
                          |LED1_Pin|LED2_Pin|LED3_Pin|mtr0_inB0_Pin
                          |mtr0_inA0_Pin|mtr0_inA1_Pin|mtr3_in0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(adc0_cs_GPIO_Port, adc0_cs_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(adc0_cnvst_GPIO_Port, adc0_cnvst_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, adc1_cs_Pin|adc1_cnvst_Pin|adc2_cs_Pin|tc4_cs_Pin
                          |SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, adc2_cnvst_Pin|tc0_cs_Pin|tc1_cs_Pin|tc2_cs_Pin
                          |tc3_cs_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, mtr2_in0_Pin|mtr2_in1_Pin|mtr2_in2_Pin|mtr3_in1_Pin
                          |mtr3_in2_Pin|mtr1_inA0_Pin|mtr1_inA1_Pin|mtr1_inB0_Pin
                          |mtr1_inB1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, mtr0_inB1_Pin|GPIO_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : en_vlv8_Pin en_vlv7_Pin en_vlv6_Pin en_vlv5_Pin
                           en_vlv4_Pin GPIO_1_Pin en_vlv0_Pin */
  GPIO_InitStruct.Pin = en_vlv8_Pin|en_vlv7_Pin|en_vlv6_Pin|en_vlv5_Pin
                          |en_vlv4_Pin|GPIO_1_Pin|en_vlv0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : en_vlv3_Pin en_vlv2_Pin en_vlv1_Pin LED0_Pin
                           LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = en_vlv3_Pin|en_vlv2_Pin|en_vlv1_Pin|LED0_Pin
                          |LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : e_hall1A_Pin e_hall1B_Pin e_hall1C_Pin adc0_eoc_Pin */
  GPIO_InitStruct.Pin = e_hall1A_Pin|e_hall1B_Pin|e_hall1C_Pin|adc0_eoc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : adc0_cs_Pin mtr0_inB1_Pin */
  GPIO_InitStruct.Pin = adc0_cs_Pin|mtr0_inB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : adc0_cnvst_Pin mtr0_inB0_Pin mtr0_inA0_Pin mtr0_inA1_Pin
                           mtr3_in0_Pin */
  GPIO_InitStruct.Pin = adc0_cnvst_Pin|mtr0_inB0_Pin|mtr0_inA0_Pin|mtr0_inA1_Pin
                          |mtr3_in0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : adc1_eoc_Pin */
  GPIO_InitStruct.Pin = adc1_eoc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(adc1_eoc_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : adc1_cs_Pin adc1_cnvst_Pin adc2_cs_Pin tc4_cs_Pin
                           SD_CS_Pin */
  GPIO_InitStruct.Pin = adc1_cs_Pin|adc1_cnvst_Pin|adc2_cs_Pin|tc4_cs_Pin
                          |SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : adc2_eoc_Pin */
  GPIO_InitStruct.Pin = adc2_eoc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(adc2_eoc_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : adc2_cnvst_Pin tc0_cs_Pin tc1_cs_Pin tc2_cs_Pin
                           tc3_cs_Pin */
  GPIO_InitStruct.Pin = adc2_cnvst_Pin|tc0_cs_Pin|tc1_cs_Pin|tc2_cs_Pin
                          |tc3_cs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_CS_Pin mtr2_in0_Pin mtr2_in1_Pin mtr2_in2_Pin
                           mtr3_in1_Pin mtr3_in2_Pin mtr1_inA0_Pin mtr1_inA1_Pin
                           mtr1_inB0_Pin mtr1_inB1_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|mtr2_in0_Pin|mtr2_in1_Pin|mtr2_in2_Pin
                          |mtr3_in1_Pin|mtr3_in2_Pin|mtr1_inA0_Pin|mtr1_inA1_Pin
                          |mtr1_inB0_Pin|mtr1_inB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : e_hall0A_Pin e_hall0B_Pin e_hall0C_Pin */
  GPIO_InitStruct.Pin = e_hall0A_Pin|e_hall0B_Pin|e_hall0C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : mtr2_pwm0_Pin mtr2_pwm1_Pin mtr2_pwm2_Pin */
  GPIO_InitStruct.Pin = mtr2_pwm0_Pin|mtr2_pwm1_Pin|mtr2_pwm2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : mtr3_pwm0_Pin mtr3_pwm1_Pin mtr3_pwm2_Pin */
  GPIO_InitStruct.Pin = mtr3_pwm0_Pin|mtr3_pwm1_Pin|mtr3_pwm2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_3_Pin */
  GPIO_InitStruct.Pin = GPIO_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
