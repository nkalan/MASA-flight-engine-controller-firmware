/*
 * autosequence.h
 *
 * Interface for controlling the engine controller's internal state machine
 * Handles automatic state transitions
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#ifndef INC_AUTOSEQUENCE_H_
#define INC_AUTOSEQUENCE_H_

#include <stdint.h>
#include "valves.h"
#include "tank_pressure_control.h"

/**
 * Struct to hold autosequence timings for convenience
 * All times are in milliseconds
 * All times are relative to the start of their respective state
 */
typedef struct {

	// Timing variables, organized by state
	// volatile variables are set from the GUI and saved to flash

	uint32_t startup_motor_start_delay_ms;

	volatile uint16_t ignition_ignitor_on_delay_ms;
	volatile uint16_t ignition_ignitor_high_duration_ms;

	volatile uint8_t hotfire_fuel_mpv_delay_ms;
	volatile uint16_t hotfire_film_cooling_on_time_ms;
	volatile uint16_t hotfire_pid_start_delay_ms;
	uint32_t hotfire_purge_off_time_ms;
	volatile uint32_t hotfire_test_duration_ms;

	uint32_t post_vent_on_time_ms;
	uint32_t post_vent_off_time_ms;
	uint32_t post_purge_off_time_ms;

	// Flag for sending commands to GSE in periodic loop
	uint8_t gse_fuel_vent_signal;
	uint8_t gse_fuel_vent_command_enable;

	uint8_t gse_fuel_vent_high_signal_sent;
	uint8_t gse_fuel_vent_low_signal_sent;

	// Time in current state; 0 if it doens't care
	uint32_t T_state;

	// Control variables
	uint8_t hotfire_lox_tank_enable_PID_control;   // 1 during active TPC
	uint8_t hotfire_fuel_tank_enable_PID_control;  // 1 during active TPC
	uint8_t startup_init_motor_pos_complete;       // Make sure motor turns to initial position
	volatile uint8_t enable_auto_aborts;                   // Toggle automatic aborts

	// Ignition + Combustion detection control variables + constants
	float ignition_ignitor_current_lower_bound;  // Threshold to detect ignitor break
	uint32_t ignition_ignitor_current_lower_bound_pass_min_detections;  // Minimum detections to read past lower bound to trigger flag
	uint16_t ignition_ignitor_current_lower_bound_pass_count;  // Live count of ignitor break current detections
	// Starts counting when ignitor goes high

	float hotfire_chamber_pres_upper_bound;  // Hard start detection
	uint32_t hotfire_chamber_pres_upper_bound_pass_min_detections;
	uint16_t hotfire_chamber_pres_upper_bound_pass_count;
	// Counts for entire Hotfire state

	float hotfire_chamber_pres_lower_bound;  // Combustion failure detection
	uint32_t hotfire_chamber_pres_lower_bound_pass_min_detections;
	uint16_t hotfire_chamber_pres_lower_bound_pass_count;
	uint32_t hotfire_chamber_pres_lower_bound_abort_start_time_ms;
	// Only counts X time after Hotfire starts

	// Outputs of ignition detection
	uint8_t ignition_ignitor_current_lower_bound_threshold_passed;
	uint8_t hotfire_chamber_pres_lower_bound_threshold_passed;
	uint8_t hotfire_chamber_pres_upper_bound_threshold_passed;

	// Start time variables - set automatically during autosequence
	volatile uint32_t startup_start_time_ms;   // Set when entering Startup
	volatile uint32_t ignition_start_time_ms;  // Set in Ignition set_state command
	uint32_t hotfire_start_time_ms;
	uint32_t post_start_time_ms;

} Autosequence_Info;


void init_autosequence_constants();


void init_autosequence_timings();


void init_tank_pressure_control_configuration();


void transmit_autosequence_gse_valve_commands();


void init_autosequence_control_variables();


void execute_autosequence();


uint32_t get_ellapsed_time_in_autosequence_state_ms();


void update_ignitor_break_detector();


void update_combustion_failure_detector();


void update_hard_start_detector();


/**
 * Called by set_state in telem.c
 * Modifies global variable STATE
 */
void manual_state_transition(uint8_t next_state);

#endif /* INC_AUTOSEQUENCE_H_ */
