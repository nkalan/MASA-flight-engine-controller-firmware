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
	uint32_t ignition_ignitor_on_time_ms;
	uint32_t ignition_ignitor_off_time_ms;

	uint32_t hotfire_fuel_on_time_ms;
	uint32_t hotfire_lox_PID_control_start_time_ms;
	uint32_t hotfire_fuel_PID_control_start_time_ms;
	uint32_t hotfire_film_cooling_on_time_ms;
	uint32_t hotfire_purge_off_time_ms;
	uint32_t hotfire_complete_time_ms;

	uint32_t post_vent_on_time_ms;
	uint32_t post_vent_off_time_ms;
	uint32_t post_purge_off_time_ms;

	// Control variables
	uint8_t hotfire_lox_tank_enable_PID_control;
	uint8_t hotfire_fuel_tank_enable_PID_control;
	uint8_t ignition_failure_shutdown_flag;

} Autosequence_Timings;


/**
 * Called by set_state in telem.c
 * Modifies global variable STATE
 */
void manual_state_transition(uint8_t next_state);

#endif /* INC_AUTOSEQUENCE_H_ */
