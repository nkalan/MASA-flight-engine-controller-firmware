/*
 * L6470.c
 *
 *  Created on: May 23, 2021
 *      Author: Nathaniel Kalantar and Samantha Liu
 *
 *  Datasheet:
 *  https://www.st.com/content/ccc/resource/technical/document/datasheet/a5/86/06/1c/fa/b2/43/db/CD00255075.pdf/files/CD00255075.pdf/_jcr_content/translations/en.CD00255075.pdf
 *
 *
 *  Note: there's a lot of manual bit shifting because each SPI command
 *  argument is left justified, and they all have different lengths
 */

#include "../inc/L6470.h"
#include <stdbool.h>

/**
 * Constants
 */
#define L6470_SPI_TIMEOUT   (0xFF)  // Arbitrary
#define L6470_TICK_DURATION (0.000000250F)  // 250 ns


/**
 * Commands
 *
 * Each command is an 8bit binary code, where the first 3 bits (b7-5) are the
 * command type, and the remaining 5 bits (b4-0) are command parameters.
 * Some commands require additional parameters, which are commented.
 *
 * Comments identify bits 7-0, with b7=MSB, b0=LSB
 * Bits are MSB first
 *
 * Datasheet pg 56
 */
#define L6470_CMD_NOP                    ((uint8_t) 0b00000000)
#define L6470_CMD_SETPARAM               ((uint8_t) 0b00000000)  // set PARAM register in b4-0
#define L6470_CMD_GETPARAM               ((uint8_t) 0b00100000)  // get PARAM register in b4-0
#define L6470_CMD_RUN                    ((uint8_t) 0b01010000)  // b0=DIR
#define L6470_CMD_STEPCLOCK              ((uint8_t) 0b01011000)  // b0=DIR
#define L6470_CMD_MOVE                   ((uint8_t) 0b01000000)  // b0=DIR
#define L6470_CMD_GOTO                   ((uint8_t) 0b01100000)  // always takes minimum path
#define L6470_CMD_GOTO_DIR               ((uint8_t) 0b01101000)  // set b0=DIR to force direction
#define L6470_CMD_GOUNTIL                ((uint8_t) 0b10000010)  // b3=ACT, b0=DIR
#define L6470_CMD_RELEASESW              ((uint8_t) 0b10010010)  // b3=ACT, b0=DIR
#define L6470_CMD_GOHOME                 ((uint8_t) 0b01110000)
#define L6470_CMD_GOMARK                 ((uint8_t) 0b01111000)
#define L6470_CMD_RESETPOS               ((uint8_t) 0b11011000)
#define L6470_CMD_RESETDEVICE            ((uint8_t) 0b11000000)
#define L6470_CMD_SOFTSTOP               ((uint8_t) 0b10110000)
#define L6470_CMD_HARDSTOP               ((uint8_t) 0b10111000)
#define L6470_CMD_SOFTHIZ                ((uint8_t) 0b10100000)
#define L6470_CMD_HARDHIZ                ((uint8_t) 0b10101000)
#define L6470_CMD_GETSTATUS              ((uint8_t) 0b11010000)


/**
 * Status register bits
 * Latched status bits stay in that state until they are read
 * Datasheet pg 55
 */
#define L6470_STATUS_BIT_HiZ           ((uint16_t)0x0001)  // Bridges are in HiZ
#define L6470_STATUS_BIT_BUSY          ((uint16_t)0x0002)  // Mirrors ~BUSY pin
#define L6470_STATUS_BIT_SW_F          ((uint16_t)0x0004)  // SW input status (low==open)
#define L6470_STATUS_BIT_SW_EVN        ((uint16_t)0x0008)  // (latched) SW input falling edge (switch turn on event)
#define L6470_STATUS_BIT_DIR           ((uint16_t)0x0010)  // 1=forward, 0=reverse
#define L6470_STATUS_BIT_MOT_STATUS_1  ((uint16_t)0x0020)  // See L6470_Motor_Status enum
#define L6470_STATUS_BIT_MOT_STATUS_0  ((uint16_t)0x0040)  // ^
#define L6470_STATUS_BIT_NOTPERF_CMD   ((uint16_t)0x0080)  // (latched) SPI command not performed
#define L6470_STATUS_BIT_WRONG_CMD     ((uint16_t)0x0100)  // (latched) SPI command doesn't exist
#define L6470_STATUS_BIT_UVLO          ((uint16_t)0x0200)  // (active low, latched) Undervoltage lockout or reset
#define L6470_STATUS_BIT_TH_WRN        ((uint16_t)0x0400)  // (latched) Thermal warning
#define L6470_STATUS_BIT_TH_SD         ((uint16_t)0x0800)  // (latched) Thermal shutdown
#define L6470_STATUS_BIT_OCD           ((uint16_t)0x1000)  // (latched) Overcurrent
#define L6470_STATUS_BIT_STEP_LOSS_A   ((uint16_t)0x2000)  // (active low, latched) Stall detected on bridge A
#define L6470_STATUS_BIT_STEP_LOSS_B   ((uint16_t)0x4000)  // (active low, latched) Stall detected on bridge B
#define L6470_STATUS_BIT_SCK_MOD       ((uint16_t)0x8000)  // (unused) step-clock mode


void L6470_SPI_CS_delay(L6470_Motor_IC *motor) {
	// Need to keep CS high >= 800ns in between SPI byte transmissions
	// Assume max HCLK=180MHz, 1 cycle=5.5555ns
	// 145 cycles required to delay that much
	// Do 150 cycles

	for (uint8_t i = 0; i < 150; i++) {
		asm("nop"); // Delay next cycle
	}
}

/**
 * Send a byte to the motor chip
 *
 * @param tx: Byte to transmit to the motor chip
 */
void L6470_SPI_transmit_byte(L6470_Motor_IC *motor, uint8_t tx) {
	HAL_GPIO_WritePin(motor->cs_base, motor->cs_pin, GPIO_PIN_RESET);
	motor->HAL_SPI_Status = HAL_SPI_Transmit(motor->hspi, &tx, 1,
			L6470_SPI_TIMEOUT);
	HAL_GPIO_WritePin(motor->cs_base, motor->cs_pin, GPIO_PIN_SET);
}

/**
 * Receive a byte from the motor chip and return its value.
 */
uint8_t L6470_SPI_receive_byte(L6470_Motor_IC *motor) {
	uint8_t rx;
	HAL_GPIO_WritePin(motor->cs_base, motor->cs_pin, GPIO_PIN_RESET);
	motor->HAL_SPI_Status = HAL_SPI_Receive(motor->hspi, &rx, 1,
			L6470_SPI_TIMEOUT);
	HAL_GPIO_WritePin(motor->cs_base, motor->cs_pin, GPIO_PIN_SET);
	return rx;
}

//---------------------End of Helper Functions----------------------

uint32_t L6470_read_register(L6470_Motor_IC *motor, uint8_t reg_addr) {
	uint8_t tx = L6470_CMD_GETPARAM | reg_addr;
	uint8_t rx[4] = {0};
	uint32_t return_val = 0;

	__disable_irq();
	L6470_SPI_transmit_byte(motor, tx);

	// All registers are >= 1 byte
	L6470_SPI_CS_delay(motor);
	rx[0] = L6470_SPI_receive_byte(motor);
	return_val = (uint32_t)rx[0];

	// Registers >= 2 byte
	if (reg_addr == L6470_PARAM_ABS_POS_ADDR
			|| reg_addr == L6470_PARAM_EL_POS_ADDR
			|| reg_addr == L6470_PARAM_MARK_ADDR
			|| reg_addr == L6470_PARAM_SPEED_ADDR
			|| reg_addr == L6470_PARAM_ACC_ADDR
			|| reg_addr == L6470_PARAM_DEC_ADDR
			|| reg_addr == L6470_PARAM_MAX_SPEED_ADDR
			|| reg_addr == L6470_PARAM_MIN_SPEED_ADDR
			|| reg_addr == L6470_PARAM_FS_SPD_ADDR
			|| reg_addr == L6470_PARAM_INT_SPEED_ADDR
			|| reg_addr == L6470_PARAM_CONFIG_ADDR
			|| reg_addr == L6470_PARAM_STATUS_ADDR) {
		L6470_SPI_CS_delay(motor);
		rx[1] = L6470_SPI_receive_byte(motor);
		return_val <<= 8;
		return_val |= (uint32_t)rx[1];
	}

	// 3 byte registers
	if (reg_addr == L6470_PARAM_ABS_POS_ADDR
			|| reg_addr == L6470_PARAM_MARK_ADDR
			|| reg_addr == L6470_PARAM_SPEED_ADDR) {
		L6470_SPI_CS_delay(motor);
		rx[2] = L6470_SPI_receive_byte(motor);
		return_val <<= 8;
		return_val |= (uint32_t)rx[2];
	}

	__enable_irq();

	return return_val;
}


void L6470_write_register(L6470_Motor_IC *motor, uint8_t reg_addr,
		uint32_t reg_val) {
	// Similar logic as L6470_read_register(), but write instead
	// User handles matching the right length of reg_val to reg_addr
	uint8_t tx = L6470_CMD_SETPARAM | reg_addr;
	uint32_t shifted_byte = 0;

	__disable_irq();
	L6470_SPI_transmit_byte(motor, tx);

	// 3 byte registers
	if (reg_addr == L6470_PARAM_ABS_POS_ADDR
			|| reg_addr == L6470_PARAM_MARK_ADDR
			|| reg_addr == L6470_PARAM_SPEED_ADDR) {
		shifted_byte = reg_val;
		shifted_byte >>= 16;
		L6470_SPI_CS_delay(motor);
		L6470_SPI_transmit_byte(motor, (uint8_t)shifted_byte);
	}

	// Registers >= 2 byte
	if (reg_addr == L6470_PARAM_ABS_POS_ADDR
			|| reg_addr == L6470_PARAM_EL_POS_ADDR
			|| reg_addr == L6470_PARAM_MARK_ADDR
			|| reg_addr == L6470_PARAM_SPEED_ADDR
			|| reg_addr == L6470_PARAM_ACC_ADDR
			|| reg_addr == L6470_PARAM_DEC_ADDR
			|| reg_addr == L6470_PARAM_MAX_SPEED_ADDR
			|| reg_addr == L6470_PARAM_MIN_SPEED_ADDR
			|| reg_addr == L6470_PARAM_FS_SPD_ADDR
			|| reg_addr == L6470_PARAM_INT_SPEED_ADDR
			|| reg_addr == L6470_PARAM_CONFIG_ADDR
			|| reg_addr == L6470_PARAM_STATUS_ADDR) {
		shifted_byte = reg_val;
		shifted_byte >>= 8;
		L6470_SPI_CS_delay(motor);
		L6470_SPI_transmit_byte(motor, (uint8_t)shifted_byte);
	}

	// All registers are >= 1 byte
	shifted_byte = reg_val;
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)shifted_byte);

	L6470_SPI_CS_delay(motor);

	__enable_irq();

	return;
}


void L6470_reset_device(L6470_Motor_IC* motor) {
	uint8_t tx = L6470_CMD_RESETDEVICE;

	__disable_irq();
	L6470_SPI_transmit_byte(motor, tx);
	L6470_SPI_CS_delay(motor);
	__enable_irq();

	return;
}


/**
 * Read the status register and update the struct's status variables
 * Datasheet pg 55
 */
void L6470_get_status(L6470_Motor_IC *motor) {

	uint8_t tx = L6470_CMD_GETSTATUS;
	uint8_t rx[2] = {0};

	__disable_irq();
	L6470_SPI_transmit_byte(motor, tx);
	L6470_SPI_CS_delay(motor);
	rx[0] = L6470_SPI_receive_byte(motor);
	L6470_SPI_CS_delay(motor);
	rx[1] = L6470_SPI_receive_byte(motor);
	__enable_irq();

	uint16_t status_reg = ((uint16_t)rx[0] << 8) | ((uint16_t)rx[1]);

	//uint32_t status_reg_read = L6470_read_register(motor, L6470_PARAM_STATUS_ADDR);

	// 1 bit statuses ("casting as bool" to avoid integer overflow)
	motor->HiZ_status         =  (status_reg & L6470_STATUS_BIT_HiZ);
	motor->BUSY_status        = 0 != (status_reg & L6470_STATUS_BIT_BUSY);
	motor->SW_F_status        = 0 != (status_reg & L6470_STATUS_BIT_SW_F);
	motor->SW_EVN_status      = 0 != (status_reg & L6470_STATUS_BIT_SW_EVN);
	motor->DIR_status         = 0 != (status_reg & L6470_STATUS_BIT_DIR);
	motor->NOTPERF_CMD_status = 0 != (status_reg & L6470_STATUS_BIT_NOTPERF_CMD);
	motor->WRONG_CMD_status   = 0 != (status_reg & L6470_STATUS_BIT_WRONG_CMD);
	motor->UVLO_status        = 0 != (status_reg & L6470_STATUS_BIT_UVLO);
	motor->TH_WRN_status      = 0 != (status_reg & L6470_STATUS_BIT_TH_WRN);
	motor->TH_SD_status       = 0 != (status_reg & L6470_STATUS_BIT_TH_SD);
	motor->OCD_status         = 0 != (status_reg & L6470_STATUS_BIT_OCD);
	motor->STEP_LOSS_A_status = 0 != (status_reg & L6470_STATUS_BIT_STEP_LOSS_A);
	motor->STEP_LOSS_B_status = 0 != (status_reg & L6470_STATUS_BIT_STEP_LOSS_B);
	motor->SCK_MOD_status     = 0 != (status_reg & L6470_STATUS_BIT_SCK_MOD);

	// 2 bit motor status
	uint8_t motor_status_1    = status_reg & L6470_STATUS_BIT_MOT_STATUS_1;
	uint8_t motor_status_0    = status_reg & L6470_STATUS_BIT_MOT_STATUS_0;

	uint8_t motor_status = (motor_status_1 << 1) | (motor_status_0);
	switch(motor_status) {
	case 0:
		motor->MOT_status = Stopped;
		break;
	case 1:
		motor->MOT_status = Acceleration;
		break;
	case 2:
		motor->MOT_status = Deceleration;
		break;
	case 3:
		motor->MOT_status = Constant_Speed;
		break;
	default:
		break;
	}
}


void L6470_init_motor(L6470_Motor_IC* motor, L6470_Stepping_Mode mode, float step_angle) {
	// Call L6470_get_status to reset FLAG
	L6470_get_status(motor);

	// Configure the stepping mode
	motor->step_mode = mode;


	// When the stepping mode is changed, the ABS_POS register is invalidated, so zero it
	L6470_zero_motor(motor);
	L6470_write_register(motor, L6470_PARAM_STEP_MODE_ADDR, mode);

	// Store the step angle in the struct
	motor->step_angle = step_angle;

	// Stop the motor in case it's moving.
	// This happens when the microcontroller resets without losing
	// power while the motor is moving.
	L6470_stop_motor(motor);

	return;
}


void L6470_set_motor_max_speed(L6470_Motor_IC* motor, float degree_per_sec) {
//	// Check register h07; default should be h41
	uint32_t max_speed = L6470_read_register(motor, L6470_PARAM_MAX_SPEED_ADDR);

	// Convert; datasheet pg 43
	// Max is 1023 step/tick = 15610 step/s = 28098 degree/s;
	// CAUTION: Because we want to compensate for this division so that the motor can run() at a speed close to max,
	// I added this +1, but this does mean that goto() runs at a faster speed than the degree_per_sec here
	uint32_t step_per_tick = (uint32_t)((degree_per_sec / 1.8 / 15.25) + 1);
	L6470_write_register(motor, L6470_PARAM_MAX_SPEED_ADDR, step_per_tick);

	//check register
	//max_speed = L6470_read_register(motor, L6470_PARAM_MAX_SPEED_ADDR);

	return;
}


void L6470_set_motor_acc_dec(L6470_Motor_IC* motor, float acc_ratio, float dec_ratio){
	uint32_t acc_step_tick_2 = (uint16_t)(4095 * acc_ratio + 0.5);
	uint32_t dec_step_tick_2 = (uint16_t)(4095 * dec_ratio + 0.5);
	L6470_write_register(motor, L6470_PARAM_ACC_ADDR, acc_step_tick_2);
	L6470_write_register(motor, L6470_PARAM_DEC_ADDR, dec_step_tick_2);

	//uint32_t check = L6470_read_register(motor, L6470_PARAM_DEC_ADDR);
}


void L6470_goto_motor_pos(L6470_Motor_IC* motor, float abs_pos_degree) {
	uint32_t abs_pos_step = 0;

	//Convert degrees to steps
	if (abs_pos_degree < 0) {
		abs_pos_step = (uint32_t)(abs_pos_degree * -1 / motor->step_angle);
		abs_pos_step -= 1;
		abs_pos_step = ~abs_pos_step;
	}
	else { // Positive (forward direction)
		abs_pos_step = (uint32_t)(abs_pos_degree / motor->step_angle);
	}

	__disable_irq();
	L6470_SPI_transmit_byte(motor, L6470_CMD_GOTO);
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)(abs_pos_step >> 16));
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)(abs_pos_step >> 8));
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)abs_pos_step);
	__enable_irq();

	return;
}


void L6470_zero_motor(L6470_Motor_IC* motor) {
	uint8_t tx = L6470_CMD_RESETPOS;

	__disable_irq();
	L6470_SPI_transmit_byte(motor, tx);
	L6470_SPI_CS_delay(motor);
	__enable_irq();

	return;
}


void L6470_goto_motor_pos_dir(L6470_Motor_IC* motor, uint8_t dir, float abs_pos_degree) {
	//Convert degrees to steps
	uint32_t abs_pos_step = (uint32_t)(abs_pos_degree / motor->step_angle);

	__disable_irq();
	L6470_SPI_transmit_byte(motor, L6470_CMD_GOTO_DIR|dir);
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)(abs_pos_step >> 16));
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)(abs_pos_step >> 8));
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)abs_pos_step);
	__enable_irq();

	// Busy

	return;
}


void L6470_run(L6470_Motor_IC* motor, uint8_t dir, float speed_deg_sec) {
	speed_deg_sec /= motor->step_angle; // Convert to step/sec
	// This formula is from datasheet pg 42. 90 degree/sec converts to ~3355 step/tick
	uint32_t speed_step_tick = (uint32_t)(speed_deg_sec * 67.108864);

	__disable_irq();
	L6470_SPI_transmit_byte(motor, L6470_CMD_RUN | dir);
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)(speed_step_tick >> 16));
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)(speed_step_tick >> 8));
	L6470_SPI_CS_delay(motor);
	L6470_SPI_transmit_byte(motor, (uint8_t)speed_step_tick);
	__enable_irq();

	return;
}


void L6470_stop_motor(L6470_Motor_IC* motor) {
	L6470_SPI_transmit_byte(motor, L6470_CMD_HARDHIZ);
	return;
}


void L6470_lock_motor(L6470_Motor_IC* motor) {
	L6470_SPI_transmit_byte(motor, L6470_CMD_HARDSTOP);
	return;
}


float L6470_get_position_deg(L6470_Motor_IC* motor) {
	float pos = 0;
	pos = (float)L6470_read_register(motor, L6470_PARAM_ABS_POS_ADDR);

	// Converting
	pos *= motor->step_angle;

	//update struct

	return pos;
}


float L6470_get_speed_steps_sec(L6470_Motor_IC* motor) {
	float spd = 0;
	spd = (float)L6470_read_register(motor, L6470_PARAM_SPEED_ADDR);

	// update struct
	motor->speed = (uint32_t)spd;

	// Converting according to datasheet pg 42
	spd = spd / 67.108864 * motor->step_angle;

	return spd;
}
