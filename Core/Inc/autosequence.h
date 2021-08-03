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

	volatile uint16_t ignition_ignitor_on_delay_ms;
	volatile uint16_t ignition_ignitor_high_duration_ms;

	volatile uint8_t hotfire_fuel_mpv_opening_delay_ms;
	volatile uint16_t hotfire_film_cooling_on_time_ms;
	volatile uint16_t hotfire_pid_start_delay_ms;
	uint32_t hotfire_purge_off_time_ms;
	volatile uint32_t hotfire_complete_time_ms;

	uint32_t post_vent_on_time_ms;
	uint32_t post_vent_off_time_ms;
	uint32_t post_purge_off_time_ms;

	// Control variables
	uint8_t ignition_failure_shutdown_flag;
	uint8_t hotfire_lox_tank_enable_PID_control;   // 1 during active TPC
	uint8_t hotfire_fuel_tank_enable_PID_control;  // 1 during active TPC

	// Start time variables - set automatically during autosequence
	volatile uint32_t ignition_start_time_ms;  // Set in Ignition set_state command
	uint32_t hotfire_start_time_ms;
	uint32_t post_start_time_ms;

} Autosequence_Info;

void init_autosequence_timings();


void init_tank_pressure_control_configuration();


void execute_autosequence();


/**
 * Called by set_state in telem.c
 * Modifies global variable STATE
 */
void manual_state_transition(uint8_t next_state);

#endif /* INC_AUTOSEQUENCE_H_ */
