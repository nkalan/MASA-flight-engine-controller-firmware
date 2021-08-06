/*
 * L6470.h
 *
 *  Created on: May 23, 2021
 *      Author: Nathaniel Kalantar and Samantha Liu
 *
 *  Datasheet:
 *  https://www.st.com/content/ccc/resource/technical/document/datasheet/a5/86/06/1c/fa/b2/43/db/CD00255075.pdf/files/CD00255075.pdf/_jcr_content/translations/en.CD00255075.pdf
 *
 */

#ifndef INC_L6470_H_
#define INC_L6470_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef HAL_SPI_MODULE_ENABLED	// Begin SPI include protection


/**
 * Register Addresses
 * Each one stores a different motor IC parameter
 * Each parameter has a different length, see datasheet
 *
 * Parameter addresses should be ORed with the GETPARAM and SETPARAM commands
 * Note that all param addresses are 5 bits. These fit into the GETPARAM and SETPARAM commands LSBs
 *
 * Datasheet pg 40
 */
#define L6470_PARAM_ABS_POS_ADDR        ((uint8_t) 0x01)  // 22 bits
#define L6470_PARAM_EL_POS_ADDR         ((uint8_t) 0x02)  // 9 bits
#define L6470_PARAM_MARK_ADDR           ((uint8_t) 0x03)  // 22 bits
#define L6470_PARAM_SPEED_ADDR          ((uint8_t) 0x04)  // 20 bits
#define L6470_PARAM_ACC_ADDR            ((uint8_t) 0x05)  // 12 bits
#define L6470_PARAM_DEC_ADDR            ((uint8_t) 0x06)  // 12 bits
#define L6470_PARAM_MAX_SPEED_ADDR      ((uint8_t) 0x07)  // 10 bits
#define L6470_PARAM_MIN_SPEED_ADDR      ((uint8_t) 0x08)  // 13 bits
#define L6470_PARAM_FS_SPD_ADDR         ((uint8_t) 0x15)  // 10 bits
#define L6470_PARAM_KVAL_HOLD_ADDR      ((uint8_t) 0x09)  // 8 bits
#define L6470_PARAM_KVAL_RUN_ADDR       ((uint8_t) 0x0A)  // 8 bits
#define L6470_PARAM_KVAL_ACC_ADDR       ((uint8_t) 0x0B)  // 8 bits
#define L6470_PARAM_KVAL_DEC_ADDR       ((uint8_t) 0x0C)  // 8 bits
#define L6470_PARAM_INT_SPEED_ADDR      ((uint8_t) 0x0D)  // 14 bits
#define L6470_PARAM_ST_SLP_ADDR         ((uint8_t) 0x0E)  // 8 bits
#define L6470_PARAM_FN_SLOP_ACC_ADDR    ((uint8_t) 0x0F)  // 8 bits
#define L6470_PARAM_FN_SLOP_DEC_ADDR    ((uint8_t) 0x10)  // 8 bits
#define L6470_PARAM_K_THERM_ADDR        ((uint8_t) 0x11)  // 4 bits
#define L6470_PARAM_ADC_OUT_ADDR        ((uint8_t) 0x12)  // 5 bits
#define L6470_PARAM_OCD_TH_ADDR         ((uint8_t) 0x13)  // 4 bits
#define L6470_PARAM_STALL_TH_ADDR       ((uint8_t) 0x14)  // 7 bits
#define L6470_PARAM_STEP_MODE_ADDR      ((uint8_t) 0x16)  // 8 bits
#define L6470_PARAM_ALARM_EN_ADDR       ((uint8_t) 0x17)  // 8 bits
#define L6470_PARAM_CONFIG_ADDR         ((uint8_t) 0x18)  // 16 bits
#define L6470_PARAM_STATUS_ADDR         ((uint8_t) 0x19)  // 16 bits

/**
 * Returned from status register
 * Datasheet pg 56
 */
typedef enum {
	Stopped = 0,
	Acceleration = 1,
	Deceleration = 2,
	Constant_Speed = 3
} L6470_Motor_Status;


/**
 * Stepping Mode
 *
 * Default mode on reset is 128th microstep.
 * When it's changed, the ABS_POS register is invalidated.
 *
 * Datasheet pg 47
 */
typedef enum {
	L6470_FULL_STEP_MODE = 0,          // (0b000)
	L6470_HALF_STEP_MODE = 1,          // (0b001)
	L6470_QUARTER_MICROSTEP_MODE = 2,  // (0b010)
	L6470_EIGHTH_MICROSTEP_MODE = 3,   // (0b011)
	L6470_16_MICROSTEP_MODE = 4,       // (0b100)
	L6470_32_MICROSTEP_MODE = 5,       // (0b101)
	L6470_64_MICROSTEP_MODE = 6,       // (0b100)
	L6470_128_MICROSTEP_MODE = 7       // (0b111)
} L6470_Stepping_Mode;


/**
 * Struct to contain motor controller information
 */
typedef struct {
	SPI_HandleTypeDef *hspi;      // SPI bus for communication, specified by user
	GPIO_TypeDef *cs_base;        // Chip select pin, specified by user
	uint16_t cs_pin;

	GPIO_TypeDef *busy_base;      // Active low pin set by motor while executing commands
	uint16_t busy_pin;

	uint32_t speed;               // motor speed, in steps/tick
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

/**
 * Accepts one of the L6470_PARAM_..._ADDR values defined above.
 * Returns the bits in the register.
 */
uint32_t L6470_read_register(L6470_Motor_IC* motor, uint8_t reg_addr);

/**
 * Writes the register. Check datasheet pg 40 for which registers are writable.
 */
void L6470_write_register(L6470_Motor_IC *motor, uint8_t reg_addr,
		uint32_t reg_val);

/**
 * Resets the device to power-up conditions.
 * Datasheet pg 21
 */
void L6470_reset_device(L6470_Motor_IC* motor);

/**
 * Send the GETSTATUS command, which returns the status register and resets the FLAG.
 * Stores the status bits into the struct.
 */
void L6470_get_status(L6470_Motor_IC* motor);

/**
 * Call L6470_get_status to reset FLAG, configure the stepping mode, and store the step angle in the struct.
 */
void L6470_init_motor(L6470_Motor_IC* motor, L6470_Stepping_Mode mode, float step_angle);

/**
 * Convert the speed to step/tick and set it
 *
 * @param degree_per_sec: the desired speed
 */
void L6470_set_motor_max_speed(L6470_Motor_IC* motor, float degree_per_sec);

/*
 * Set the acceleration and deceleration registers.
 * Dafault is 2008 steps/tick^2, max is 4095, min is 1.
 * In order to change the acceleration to 80% of the max, for example, pass in 0.8 for acc_ratio.
 *
 * Datasheet pg 40 and 42.
 */
void L6470_set_motor_acc_dec(L6470_Motor_IC* motor, float acc_ratio, float dec_ratio);


/**
 * Go to an absolute position (degrees) USING THE SHORTEST PATH
 *
 * @param abs_pos_degree: The absolute position in degrees to go to.
 */
void L6470_goto_motor_pos(L6470_Motor_IC* motor, float abs_pos_degree);

/**
 * Send the command to reset the absolute position register
 */
void L6470_zero_motor(L6470_Motor_IC* motor);

/**
 * Produces a motion in dir and at speed.
 *
 * @param dir: 1 for forwards and 0 for reverse.
 * @param speed: as expressed in degrees per second. Should be within the bounds of min_ and max_speed.
 *
 * Datasheet pg 60
 */
void L6470_run(L6470_Motor_IC* motor, uint8_t dir, float speed_deg_sec);

/**
 * Go to an absolute position (degrees) IN THE SPECIFIED DIRECTION
 *
 * @param abs_pos_degree: The absolute position in degrees to go to.
 * @param dir: 1 for forwards and 0 for reverse.
 * Basically not useful because absolute position can be negative.
 *
 * Datasheet pg 62
 *
 */
void L6470_goto_motor_pos_dir(L6470_Motor_IC* motor, uint8_t dir, float abs_pos_degree);

/**
 * Send the HardStop command immediately stop and lock the motor
 *
 * Datasheet pg 65
 */
void L6470_lock_motor(L6470_Motor_IC* motor);

/**
 * Send the HardHiZ command to the motor to immediately
 * disable the power bridges
 *
 * Datasheet pg 65
 */
void L6470_stop_motor(L6470_Motor_IC* motor);

/*
 * Reads the absolute position register and converts it from steps to degrees.
 * Datasheet pg 41.
 */
float L6470_get_position_deg(L6470_Motor_IC* motor);

/*
 * Reads the current speed and converts it from steps/tick to steps/sec.
 * Datasheet pg 42.
 */
float L6470_get_speed_steps_sec(L6470_Motor_IC* motor);


#endif /* HAL_SPI_MODULE_ENABLED */
#endif /* INC_L6470_H_ */
