# L6470 stepper motor IC library User Guide (Last updated Jul 25 2021)

This repository contains the MASA firmware library for the L6470 stepper motor chip.

[L6470 Datasheet](https://www.st.com/content/ccc/resource/technical/document/datasheet/a5/86/06/1c/fa/b2/43/db/CD00255075.pdf/files/CD00255075.pdf/_jcr_content/translations/en.CD00255075.pdf)

[EVAL6470H dev board Datasheet](https://www.st.com/en/motor-drivers/l6470.html)

[STM32 firmware library for dSPIN L6470](https://www.st.com/en/embedded-software/stsw-spin004.html)

[SM-17HS4023 Motor Datasheet (for connecting the motor)](https://html.alldatasheet.com/html-pdf/1179365/ETC/SM-17HS4023/139/2/SM-17HS4023.html)

## Functions and procedure
* `L6470_reset_device` resets the chip to its factory configurations (the defaults of the registers can be found on datasheet pg 40).
* `L6470_get_status` sends the GETSTATUS command, which returns the status register and resets the FLAG.
* `L6470_init_motor` calls L6470_get_status to reset FLAG, configure the stepping mode, and store the step angle in the struct.
* `L6470_set_motor_max_speed` sets the max speed. This is the speed that goto runs at.
* `L6470_goto_motor_pos(L6470_Motor_IC* motor, float abs_pos_degree)`
* `L6470_zero_motor` zeros the absolute position register.
* `L6470_run(L6470_Motor_IC* motor, uint8_t dir, float speed_deg_sec)` runs at a constant speed. After delaying for a certain amount of time, a stop function can be called.
* `L6470_hard_stop`
* `L6470_soft_stop`
* `L6470_get_position_deg`
* `L6470_get_speed_steps_sec`
* `L6470_read_register` and `L6470_write_register`.

## L6470_Motor_IC Struct
```
typedef struct {
  SPI_HandleTypeDef *hspi;      // SPI bus for communication, specified by user
  GPIO_TypeDef *cs_base;        // Chip select pin, specified by user
  uint16_t cs_pin;

  GPIO_TypeDef *busy_base;      // Active low pin set by motor while executing commands
  uint16_t busy_pin;

  uint16_t speed;               // motor speed, in steps/tick
  // bounded by MIN_SPEED and MAX_SPEED

  // HAL SPI status gets updated after every SPI transmission
  HAL_StatusTypeDef HAL_SPI_Status;

  L6470_Stepping_Mode step_mode;

  // Status bits that get updated when the STATUS register is read
  L6470_Motor_Status MOT_status;
  uint8_t HiZ_status;
  uint8_t SW_F_status;  // unused
  uint8_t SW_EVN_status;  // unused
  uint8_t DIR_status;
  uint8_t NOTPERF_CMD_status;
  uint8_t WRONG_CMD_status;
  uint8_t UVLO_status;
  uint8_t TH_WRN_status;
  uint8_t TH_SD_status;
  uint8_t OCD_status;
  uint8_t STEP_LOSS_A_status;
  uint8_t STEP_LOSS_B_status;
  uint8_t SCK_MOD_status;  // unused

  // Should be set by interrupts, not by the status register
  volatile uint8_t BUSY_status;

  float step_angle;  // degrees per step? I think our motors are 1.8, but don't assume that here

} L6470_Motor_IC;
```

## SPI configuretion
* Frame Format: Motorola
* Data Size: 8 bits
* First Bit: MSB First
* Maximum Serial Clock Frequency: 5MHz

* Configuration for STM32CudeIDE:
* Prescaler: 16
* CPOL: High
* CPHA: 2 Edge

## Speed and its conversions
Starting on datasheet pg. 42, there are a bunch of formulas for converting between step/tick and step/s, which is what the chip uses. For a more user-friendly interface, we use degree/sec. This is where the step angle (degrees per step), as pass into `L6470_init_motor`, comes into play.
- `L6470_goto_motor_pos` runs at the maximum speed, which is in register h07; it is default h41 (991.8 step/s) and can be changed 
* To convert: step/tick = deg_sec / 1.8 / 15.25 (which is 27.45).
* Max is 1023 step/tick = 15610 step/s = 28098 degree/s (datasheet)

## Testing and sample code
In main.c, make sure to ` #include "L6470.h"`.
Initialize the motor:
```
/* USER CODE BEGIN 2 */
mot.hspi = &hspi2;
mot.cs_base = SPI2_CS_GPIO_Port;
mot.cs_pin = SPI2_CS_Pin;
HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
HAL_Delay(1);

L6470_reset_device(&mot);

L6470_init_motor(&mot, L6470_FULL_STEP_MODE, 1.8);
```
Set speed, acceleration, and deceleration:
```
L6470_set_motor_max_speed(&mot, 360);
L6470_set_motor_acc_dec(&mot, 0.5, 0.5);
```
Before the while loop, run() can be called to have the motor start running in a constant speed:
```
L6470_run(&mot, 1, 90);
```
In the while loop, multiple functions can be tested:
```
// Read Write
    L6470_reset_device(&mot);
    status_reg_read = L6470_read_register(&mot, L6470_PARAM_STEP_MODE_ADDR);
    L6470_write_register(&mot, L6470_PARAM_STEP_MODE_ADDR,      L6470_FULL_STEP_MODE);
    status_reg_read = L6470_read_register(&mot, L6470_PARAM_STEP_MODE_ADDR);

// Get Status
    L6470_get_status(&mot);
    HAL_Delay(2);

// Consecutive Run
    L6470_run(&mot, 1, 1080);
    HAL_Delay(3000);

    L6470_run(&mot, 0, 360);
    HAL_Delay(3000);

// Soft and hard stop
    L6470_hard_stop(&mot);
    HAL_Delay(1000);

// GOTO and zero
    L6470_goto_motor_pos(&mot, 1800);
    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == 0) {} // This is our BUSY pin
    HAL_Delay(3000);

    L6470_goto_motor_pos(&mot, 0);
    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == 0) {}
    HAL_Delay(3000);

    L6470_zero_motor(&mot);
    HAL_Delay(1);
```
