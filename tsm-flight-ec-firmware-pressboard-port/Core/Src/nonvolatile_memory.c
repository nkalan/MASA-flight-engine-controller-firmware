/*
 * flash_constants.c
 *
 *  Created on: Jul 30, 2021
 *      Author: natha
 */

#include <stdint.h>

#include "nonvolatile_memory.h"
#include "globals.h"
#include "constants.h"
#include "tank_pressure_control.h"
#include "autosequence.h"
#include "calibrations.h"
#include "W25N01GV.h"

// Define byte lengths of each memory segment
// This makes the addresses and variables easier to modify
#define NVM_PARITY_BIT_NUM_BYTES              (1)
#define NVM_PID_GAINS_NUM_BYTES               (NUM_TANKS*3*2)  // 3 gains, 2 bytes each
#define NVM_TPC_TARGET_PRES_NUM_BYTES         (NUM_TANKS*4)
#define NVM_CTRL_VLV_HIGH_PCNT_NUM_BYTES      (NUM_TANKS*4)
#define NVM_CTRL_VLV_LOW_PCNT_NUM_BYTES       (NUM_TANKS*4)
#define NVM_PT_AMBIENTS_NUM_BYTES             (NUM_PTS*4)
#define NVM_POT_AMBIENTS_NUM_BYTES            (NUM_POTS*4)
#define NVM_TANK_ENABLE_NUM_BYTES             (NUM_TANKS*1)  // 1 byte each
#define NVM_TEST_DURATION_NUM_BYTES           (4)
#define NVM_IGNITOR_ON_DELAY_NUM_BYTES        (2)
#define NVM_IGNITOR_HIGH_DURATION_NUM_BYTES   (2)
#define NVM_FUEL_MPV_OPENING_DELAY_NUM_BYTES  (1)  // No longer than  0.255s
#define NVM_FILM_COOLING_ON_TIME_NUM_BYTES    (2)
#define NVM_PID_DELAY_NUM_BYTES               (2)  // No longer than 65.535s
#define NVM_INIT_POS_DEG_CORR_FAC_NUM_BYTES   (4)
#define NVM_PT_UPPER_VOLTAGE_NUM_BYTES        (NUM_PTS*2)
#define NVM_PT_LOWER_VOLTAGE_NUM_BYTES        (NUM_PTS*2)
#define NVM_PT_PRESSURE_RANGE_NUM_BYTES       (NUM_PTS*2)
#define NVM_AUTO_ABORT_NUM_BYTES              (1)

// Define variable addresses based on their lengths and order
#define NVM_FLASH_PAGE_NUM               (0)
#define NVM_PARITY_BIT_ADDR              (0)
#define NVM_PID_GAINS_ADDR               (NVM_PARITY_BIT_ADDR + NVM_PARITY_BIT_NUM_BYTES)
#define NVM_TPC_TARGET_PRES_ADDR         (NVM_PID_GAINS_ADDR + NVM_PID_GAINS_NUM_BYTES)
#define NVM_CTRL_VLV_HIGH_PRES_ADDR      (NVM_TPC_TARGET_PRES_ADDR + NVM_TPC_TARGET_PRES_NUM_BYTES)
#define NVM_CTRL_VLV_LOW_PRES_ADDR       (NVM_CTRL_VLV_HIGH_PRES_ADDR + NVM_CTRL_VLV_HIGH_PCNT_NUM_BYTES)
#define NVM_PT_AMBIENTS_ADDR             (NVM_CTRL_VLV_LOW_PRES_ADDR + NVM_CTRL_VLV_LOW_PCNT_NUM_BYTES)
#define NVM_POT_AMBIENTS_ADDR            (NVM_PT_AMBIENTS_ADDR + NVM_PT_AMBIENTS_NUM_BYTES)
#define NVM_TANK_ENABLE_ADDR             (NVM_POT_AMBIENTS_ADDR + NVM_POT_AMBIENTS_NUM_BYTES)
#define NVM_TEST_DURATION_ADDR           (NVM_TANK_ENABLE_ADDR + NVM_TANK_ENABLE_NUM_BYTES)
#define NVM_IGNITOR_ON_DELAY_ADDR        (NVM_TEST_DURATION_ADDR + NVM_TEST_DURATION_NUM_BYTES)
#define NVM_IGNITOR_HIGH_DURATION_ADDR   (NVM_IGNITOR_ON_DELAY_ADDR + NVM_IGNITOR_ON_DELAY_NUM_BYTES)
#define NVM_FUEL_MPV_OPENING_DELAY_ADDR  (NVM_IGNITOR_HIGH_DURATION_ADDR + NVM_IGNITOR_HIGH_DURATION_NUM_BYTES)
#define NVM_FILM_COOLING_ON_TIME_ADDR    (NVM_FUEL_MPV_OPENING_DELAY_ADDR + NVM_FUEL_MPV_OPENING_DELAY_NUM_BYTES)
#define NVM_PID_DELAY_ADDR               (NVM_FILM_COOLING_ON_TIME_ADDR + NVM_FILM_COOLING_ON_TIME_NUM_BYTES)
#define NVM_INIT_POS_DEG_CORR_FAC_ADDR   (NVM_PID_DELAY_ADDR + NVM_PID_DELAY_NUM_BYTES)
#define NVM_PT_UPPER_VOLTAGE_ADDR        (NVM_INIT_POS_DEG_CORR_FAC_ADDR + NVM_INIT_POS_DEG_CORR_FAC_NUM_BYTES)
#define NVM_PT_LOWER_VOLTAGE_ADDR        (NVM_PT_UPPER_VOLTAGE_ADDR + NVM_PT_UPPER_VOLTAGE_NUM_BYTES)
#define NVM_PT_PRESSURE_RANGE_ADDR       (NVM_PT_LOWER_VOLTAGE_ADDR + NVM_PT_LOWER_VOLTAGE_NUM_BYTES)
#define NVM_AUTO_ABORT_ADDR              (NVM_PT_PRESSURE_RANGE_ADDR + NVM_PT_PRESSURE_RANGE_NUM_BYTES)

// Total size of variables
#define NVM_BUFFER_SZ    (NVM_PARITY_BIT_NUM_BYTES \
		+ NVM_PID_GAINS_NUM_BYTES \
		+ NVM_TPC_TARGET_PRES_NUM_BYTES \
		+ NVM_CTRL_VLV_HIGH_PCNT_NUM_BYTES \
		+ NVM_CTRL_VLV_LOW_PCNT_NUM_BYTES \
		+ NVM_PT_AMBIENTS_NUM_BYTES \
		+ NVM_POT_AMBIENTS_NUM_BYTES \
		+ NVM_TANK_ENABLE_NUM_BYTES \
		+ NVM_TEST_DURATION_NUM_BYTES \
		+ NVM_IGNITOR_ON_DELAY_NUM_BYTES \
		+ NVM_IGNITOR_HIGH_DURATION_ADDR \
		+ NVM_FUEL_MPV_OPENING_DELAY_NUM_BYTES \
		+ NVM_FILM_COOLING_ON_TIME_ADDR \
		+ NVM_PID_DELAY_NUM_BYTES \
		+ NVM_INIT_POS_DEG_CORR_FAC_ADDR \
		+ NVM_PT_UPPER_VOLTAGE_NUM_BYTES \
		+ NVM_PT_LOWER_VOLTAGE_NUM_BYTES \
		+ NVM_PT_PRESSURE_RANGE_NUM_BYTES \
		+ NVM_AUTO_ABORT_NUM_BYTES)


extern W25N01GV_Flash flash;
extern Autosequence_Info autosequence;
extern TPC_Info tanks[NUM_TANKS];
extern float init_motor_pos_deg_correction_factor;

uint8_t nonvolatile_memory_buffer[NVM_BUFFER_SZ];


uint8_t read_nonvolatile_variables() {
	__disable_irq();  // Trying to stop a bug that corrupts NVM

	// Read nonvolatile memory buffer from flash
	read_reserved_flash_page(&flash, NVM_FLASH_PAGE_NUM, nonvolatile_memory_buffer,
			NVM_BUFFER_SZ);

	// save_nonvolatile_variables() always writes a 0 in the first byte
	// because the flash default byte is nonzero. If this function reads
	// a nonzero number, then all other variables are invalid.
	if (nonvolatile_memory_buffer[NVM_PARITY_BIT_ADDR] != 0) {
		return 0;
	}

	// Read in all variables
	// PID gains
	tanks[0].K_p = ((nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 1] << 8)) / 100.0F;
	tanks[0].K_i = ((nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 2] << 0) |
			(nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 3] << 8)) / 100.0F;
	tanks[0].K_d = ((nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 4] << 0) |
			(nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 5] << 8)) / 100.0F;
	tanks[1].K_p = ((nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 6] << 0) |
			(nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 7] << 8)) / 100.0F;
	tanks[1].K_i = ((nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 8] << 0) |
			(nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 9] << 8)) / 100.0F;
	tanks[1].K_d = ((nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 10] << 0) |
			(nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 11] << 8)) / 100.0F;

	// Target pressure setpoints
	tanks[0].target_pres = ((nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 1] << 8) |
			(nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 2] << 16) |
			(nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 3] << 24)) / 100.0F;
	tanks[1].target_pres = ((nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 4] << 0) |
			(nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 5] << 8) |
			(nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 6] << 16) |
			(nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 7] << 24)) / 100.0F;

	// Control valve pressure thresholds
	tanks[0].PID_ctrl_vlv_high_pres = ((nonvolatile_memory_buffer[NVM_CTRL_VLV_HIGH_PRES_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_CTRL_VLV_HIGH_PRES_ADDR + 1] << 8)) / 10.0F;
	tanks[1].PID_ctrl_vlv_high_pres = ((nonvolatile_memory_buffer[NVM_CTRL_VLV_HIGH_PRES_ADDR + 2] << 0) |
			(nonvolatile_memory_buffer[NVM_CTRL_VLV_HIGH_PRES_ADDR + 3] << 8)) / 10.0F;

	tanks[0].PID_ctrl_vlv_low_pres = ((nonvolatile_memory_buffer[NVM_CTRL_VLV_LOW_PRES_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_CTRL_VLV_LOW_PRES_ADDR + 1] << 8)) / 10.0F;
	tanks[1].PID_ctrl_vlv_low_pres = ((nonvolatile_memory_buffer[NVM_CTRL_VLV_LOW_PRES_ADDR + 2] << 0) |
			(nonvolatile_memory_buffer[NVM_CTRL_VLV_LOW_PRES_ADDR + 3] << 8)) / 10.0F;

	// Ambient pressures
	for (uint8_t i = 0; i < NUM_PTS; i++) {
		pt_ambients[i] = ((nonvolatile_memory_buffer[NVM_PT_AMBIENTS_ADDR + 4*i + 0] << 0) |
				(nonvolatile_memory_buffer[NVM_PT_AMBIENTS_ADDR + 4*i + 1] << 8) |
				(nonvolatile_memory_buffer[NVM_PT_AMBIENTS_ADDR + 4*i + 2] << 16) |
				(nonvolatile_memory_buffer[NVM_PT_AMBIENTS_ADDR + 4*i + 3] << 24)) / 10.0F;
	}

	// Potentiometer ambients
	for (uint8_t i = 0; i < NUM_POTS; i++) {
		pot_ambients[i] = ((nonvolatile_memory_buffer[NVM_POT_AMBIENTS_ADDR + 4*i + 0] << 0) |
				(nonvolatile_memory_buffer[NVM_POT_AMBIENTS_ADDR + 4*i + 1] << 8) |
				(nonvolatile_memory_buffer[NVM_POT_AMBIENTS_ADDR + 4*i + 2] << 16) |
				(nonvolatile_memory_buffer[NVM_POT_AMBIENTS_ADDR + 4*i + 3] << 24)) / 10.0F;
	}

	// Tank enable flags
	tanks[0].tank_enable = nonvolatile_memory_buffer[NVM_TANK_ENABLE_ADDR + 0];
	tanks[1].tank_enable = nonvolatile_memory_buffer[NVM_TANK_ENABLE_ADDR + 1];

	// Autosequence timings
	autosequence.hotfire_test_duration_ms = ((nonvolatile_memory_buffer[NVM_TEST_DURATION_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_TEST_DURATION_ADDR + 1] << 8) |
			(nonvolatile_memory_buffer[NVM_TEST_DURATION_ADDR + 2] << 16) |
			(nonvolatile_memory_buffer[NVM_TEST_DURATION_ADDR + 3] << 24));

	autosequence.ignition_ignitor_on_delay_ms = ((nonvolatile_memory_buffer[NVM_IGNITOR_ON_DELAY_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_IGNITOR_ON_DELAY_ADDR + 1] << 8));

	autosequence.ignition_ignitor_high_duration_ms = ((nonvolatile_memory_buffer[NVM_IGNITOR_HIGH_DURATION_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_IGNITOR_HIGH_DURATION_ADDR + 1] << 8));

	autosequence.hotfire_fuel_mpv_delay_ms = nonvolatile_memory_buffer[NVM_FUEL_MPV_OPENING_DELAY_ADDR];

	autosequence.hotfire_film_cooling_on_time_ms = ((nonvolatile_memory_buffer[NVM_FILM_COOLING_ON_TIME_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_FILM_COOLING_ON_TIME_ADDR + 1] << 8));

	autosequence.hotfire_pid_start_delay_ms = ((nonvolatile_memory_buffer[NVM_PID_DELAY_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_PID_DELAY_ADDR + 1] << 8));

	// Initial motor position factor
	init_motor_pos_deg_correction_factor = ((nonvolatile_memory_buffer[NVM_INIT_POS_DEG_CORR_FAC_ADDR + 0] << 0) |
			(nonvolatile_memory_buffer[NVM_INIT_POS_DEG_CORR_FAC_ADDR + 1] << 8) |
			(nonvolatile_memory_buffer[NVM_INIT_POS_DEG_CORR_FAC_ADDR + 2] << 16) |
			(nonvolatile_memory_buffer[NVM_INIT_POS_DEG_CORR_FAC_ADDR + 3] << 24)) / 10000.0F;

	// Pressure transducer calibrations
	for (uint8_t i = 0; i < NUM_PTS; i++) {
		pt_cal_lower_voltage[i] = ((nonvolatile_memory_buffer[NVM_PT_LOWER_VOLTAGE_ADDR + 2*i + 0] << 0) |
				(nonvolatile_memory_buffer[NVM_PT_LOWER_VOLTAGE_ADDR + 2*i + 1] << 8)) / 10.0F;
		pt_cal_upper_voltage[i] = ((nonvolatile_memory_buffer[NVM_PT_UPPER_VOLTAGE_ADDR + 2*i + 0] << 0) |
				(nonvolatile_memory_buffer[NVM_PT_UPPER_VOLTAGE_ADDR + 2*i + 1] << 8)) / 10.0F;
		pt_cal_upper_pressure[i] = ((nonvolatile_memory_buffer[NVM_PT_PRESSURE_RANGE_ADDR + 2*i + 0] << 0) |
				(nonvolatile_memory_buffer[NVM_PT_PRESSURE_RANGE_ADDR + 2*i + 1] << 8));
	}

	// Autosequence automatic abort enable
	autosequence.enable_auto_aborts = nonvolatile_memory_buffer[NVM_AUTO_ABORT_ADDR];

	__enable_irq();  // Trying to stop a bug that corrupts NVM

	// Successful read
	return 1;
}


uint8_t save_nonvolatile_variables() {
	__disable_irq();  // Trying to stop a bug that corrupts NVM

	// First byte should always be 0; see read_nonvolatile_variables();
	nonvolatile_memory_buffer[NVM_PARITY_BIT_ADDR] = 0;

	// PID gains
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 0] = ((uint16_t) (tanks[0].K_p * 100.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 1] = ((uint16_t) (tanks[0].K_p * 100.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 2] = ((uint16_t) (tanks[0].K_i * 100.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 3] = ((uint16_t) (tanks[0].K_i * 100.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 4] = ((uint16_t) (tanks[0].K_d * 100.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 5] = ((uint16_t) (tanks[0].K_d * 100.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 6] = ((uint16_t) (tanks[1].K_p * 100.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 7] = ((uint16_t) (tanks[1].K_p * 100.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 8] = ((uint16_t) (tanks[1].K_i * 100.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 9] = ((uint16_t) (tanks[1].K_i * 100.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 10] = ((uint16_t) (tanks[1].K_d * 100.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_PID_GAINS_ADDR + 11] = ((uint16_t) (tanks[1].K_d * 100.0F)) >> 8;

	// Target pressure setpoints
	nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 0] = ((int32_t) (tanks[0].target_pres * 100.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 1] = ((int32_t) (tanks[0].target_pres * 100.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 2] = ((int32_t) (tanks[0].target_pres * 100.0F)) >> 16;
	nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 3] = ((int32_t) (tanks[0].target_pres * 100.0F)) >> 24;
	nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 4] = ((int32_t) (tanks[1].target_pres * 100.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 5] = ((int32_t) (tanks[1].target_pres * 100.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 6] = ((int32_t) (tanks[1].target_pres * 100.0F)) >> 16;
	nonvolatile_memory_buffer[NVM_TPC_TARGET_PRES_ADDR + 7] = ((int32_t) (tanks[1].target_pres * 100.0F)) >> 24;

	// Control valve threshold pressures
	nonvolatile_memory_buffer[NVM_CTRL_VLV_HIGH_PRES_ADDR + 0] = ((int16_t) (tanks[0].PID_ctrl_vlv_high_pres * 10.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_CTRL_VLV_HIGH_PRES_ADDR + 1] = ((int16_t) (tanks[0].PID_ctrl_vlv_high_pres * 10.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_CTRL_VLV_HIGH_PRES_ADDR + 2] = ((int16_t) (tanks[1].PID_ctrl_vlv_high_pres * 10.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_CTRL_VLV_HIGH_PRES_ADDR + 3] = ((int16_t) (tanks[1].PID_ctrl_vlv_high_pres * 10.0F)) >> 8;

	nonvolatile_memory_buffer[NVM_CTRL_VLV_LOW_PRES_ADDR + 0] = ((int16_t) (tanks[0].PID_ctrl_vlv_low_pres * 10.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_CTRL_VLV_LOW_PRES_ADDR + 1] = ((int16_t) (tanks[0].PID_ctrl_vlv_low_pres * 10.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_CTRL_VLV_LOW_PRES_ADDR + 2] = ((int16_t) (tanks[1].PID_ctrl_vlv_low_pres * 10.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_CTRL_VLV_LOW_PRES_ADDR + 3] = ((int16_t) (tanks[1].PID_ctrl_vlv_low_pres * 10.0F)) >> 8;

	// Pressure ambients
	for (uint8_t i = 0; i < NUM_PTS; i++) {
		nonvolatile_memory_buffer[NVM_PT_AMBIENTS_ADDR + 4*i + 0] = ((int32_t) (pt_ambients[i] * 10.0F)) >> 0;
		nonvolatile_memory_buffer[NVM_PT_AMBIENTS_ADDR + 4*i + 1] = ((int32_t) (pt_ambients[i] * 10.0F)) >> 8;
		nonvolatile_memory_buffer[NVM_PT_AMBIENTS_ADDR + 4*i + 2] = ((int32_t) (pt_ambients[i] * 10.0F)) >> 16;
		nonvolatile_memory_buffer[NVM_PT_AMBIENTS_ADDR + 4*i + 3] = ((int32_t) (pt_ambients[i] * 10.0F)) >> 24;
	}

	// Potentiometer ambients
	for (uint8_t i = 0; i < NUM_POTS; i++) {
		nonvolatile_memory_buffer[NVM_POT_AMBIENTS_ADDR + 4*i + 0] = ((int32_t) (pot_ambients[i] * 10.0F)) >> 0;
		nonvolatile_memory_buffer[NVM_POT_AMBIENTS_ADDR + 4*i + 1] = ((int32_t) (pot_ambients[i] * 10.0F)) >> 8;
		nonvolatile_memory_buffer[NVM_POT_AMBIENTS_ADDR + 4*i + 2] = ((int32_t) (pot_ambients[i] * 10.0F)) >> 16;
		nonvolatile_memory_buffer[NVM_POT_AMBIENTS_ADDR + 4*i + 3] = ((int32_t) (pot_ambients[i] * 10.0F)) >> 24;
	}

	// Tank enable flags
	nonvolatile_memory_buffer[NVM_TANK_ENABLE_ADDR + 0] = tanks[0].tank_enable;
	nonvolatile_memory_buffer[NVM_TANK_ENABLE_ADDR + 1] = tanks[1].tank_enable;

	// Autosequence timings
	// Hotfire duration
	nonvolatile_memory_buffer[NVM_TEST_DURATION_ADDR + 0] = ((uint32_t) (autosequence.hotfire_test_duration_ms)) >> 0;
	nonvolatile_memory_buffer[NVM_TEST_DURATION_ADDR + 1] = ((uint32_t) (autosequence.hotfire_test_duration_ms)) >> 8;
	nonvolatile_memory_buffer[NVM_TEST_DURATION_ADDR + 2] = ((uint32_t) (autosequence.hotfire_test_duration_ms)) >> 16;
	nonvolatile_memory_buffer[NVM_TEST_DURATION_ADDR + 3] = ((uint32_t) (autosequence.hotfire_test_duration_ms)) >> 24;

	// Ignitor on delay
	nonvolatile_memory_buffer[NVM_IGNITOR_ON_DELAY_ADDR + 0] = ((uint16_t) (autosequence.ignition_ignitor_on_delay_ms)) >> 0;
	nonvolatile_memory_buffer[NVM_IGNITOR_ON_DELAY_ADDR + 1] = ((uint16_t) (autosequence.ignition_ignitor_on_delay_ms)) >> 8;

	// Ignitor high duration
	nonvolatile_memory_buffer[NVM_IGNITOR_HIGH_DURATION_ADDR + 0] = ((uint16_t) (autosequence.ignition_ignitor_high_duration_ms)) >> 0;
	nonvolatile_memory_buffer[NVM_IGNITOR_HIGH_DURATION_ADDR + 1] = ((uint16_t) (autosequence.ignition_ignitor_high_duration_ms)) >> 8;

	// Fuel MPV on delay
	nonvolatile_memory_buffer[NVM_FUEL_MPV_OPENING_DELAY_ADDR + 0] = autosequence.hotfire_fuel_mpv_delay_ms;

	// Film cooling on time
	nonvolatile_memory_buffer[NVM_FILM_COOLING_ON_TIME_ADDR + 0] = ((uint16_t) (autosequence.hotfire_film_cooling_on_time_ms)) >> 0;
	nonvolatile_memory_buffer[NVM_FILM_COOLING_ON_TIME_ADDR + 1] = ((uint16_t) (autosequence.hotfire_film_cooling_on_time_ms)) >> 8;

	// PID start delay
	nonvolatile_memory_buffer[NVM_PID_DELAY_ADDR + 0] = ((uint16_t) (autosequence.hotfire_pid_start_delay_ms)) >> 0;
	nonvolatile_memory_buffer[NVM_PID_DELAY_ADDR + 1] = ((uint16_t) (autosequence.hotfire_pid_start_delay_ms)) >> 8;

	// Initial motor position correction factor
	nonvolatile_memory_buffer[NVM_INIT_POS_DEG_CORR_FAC_ADDR + 0] = ((uint32_t) (init_motor_pos_deg_correction_factor * 10000.0F)) >> 0;
	nonvolatile_memory_buffer[NVM_INIT_POS_DEG_CORR_FAC_ADDR + 1] = ((uint32_t) (init_motor_pos_deg_correction_factor * 10000.0F)) >> 8;
	nonvolatile_memory_buffer[NVM_INIT_POS_DEG_CORR_FAC_ADDR + 2] = ((uint32_t) (init_motor_pos_deg_correction_factor * 10000.0F)) >> 16;
	nonvolatile_memory_buffer[NVM_INIT_POS_DEG_CORR_FAC_ADDR + 3] = ((uint32_t) (init_motor_pos_deg_correction_factor * 10000.0F)) >> 24;

	// Pressure transducer calibrations
	for (uint8_t i = 0; i < NUM_PTS; i++) {
		// Lower output voltage
		nonvolatile_memory_buffer[NVM_PT_LOWER_VOLTAGE_ADDR + 2*i + 0] = ((uint16_t) (pt_cal_lower_voltage[i] * 10.0F)) >> 0;
		nonvolatile_memory_buffer[NVM_PT_LOWER_VOLTAGE_ADDR + 2*i + 1] = ((uint16_t) (pt_cal_lower_voltage[i] * 10.0F)) >> 8;

		// Upper output voltage
		nonvolatile_memory_buffer[NVM_PT_UPPER_VOLTAGE_ADDR + 2*i + 0] = ((uint16_t) (pt_cal_upper_voltage[i] * 10.0F)) >> 0;
		nonvolatile_memory_buffer[NVM_PT_UPPER_VOLTAGE_ADDR + 2*i + 1] = ((uint16_t) (pt_cal_upper_voltage[i] * 10.0F)) >> 8;

		// Pressure range
		nonvolatile_memory_buffer[NVM_PT_PRESSURE_RANGE_ADDR + 2*i + 0] = ((uint16_t) (pt_cal_upper_pressure[i])) >> 0;
		nonvolatile_memory_buffer[NVM_PT_PRESSURE_RANGE_ADDR + 2*i + 1] = ((uint16_t) (pt_cal_upper_pressure[i])) >> 8;
	}

	// Autosequence automatic abort enable
	nonvolatile_memory_buffer[NVM_AUTO_ABORT_ADDR] = autosequence.enable_auto_aborts;

	// Overwrite previous values in flash
	erase_reserved_flash_pages(&flash);
	write_reserved_flash_page(&flash, NVM_FLASH_PAGE_NUM, nonvolatile_memory_buffer,
			NVM_BUFFER_SZ);

	// Ensure they got saved
	read_nonvolatile_variables();

	__enable_irq();  // Trying to stop a bug that corrupts NVM
	return 1;
}
