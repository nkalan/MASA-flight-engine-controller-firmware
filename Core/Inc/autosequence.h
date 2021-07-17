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
#include "constants.h"  // sensor/actuator mappings
#include "valves.h"


#define BIGBERTHA
//#define PT163


/**
 * Struct to hold autosequence timings for convenience
 * All times are in milliseconds
 * All times are relative to the start of their respective state
 */
typedef struct {
	uint32_t ignition_ignitor_on_delay_ms;
	uint32_t ignition_ignitor_off_delay_ms;

	uint32_t hotfire_mpv_delay_ms;  // need a variable for which one to open first
	uint32_t hotfire_film_cooling_on_delay_ms;
	uint32_t hotfire_purge_off_delay_ms;

	uint32_t post_fuel_off_delay_ms;
	uint32_t post_vent_on_delay_ms;
	uint32_t post_vent_off_delay_ms;
	uint32_t post_purge_off_delay_ms;

	uint8_t lox_tank_enable_PID_control;
	uint8_t fuel_tank_enable_PID_control;
	uint8_t ignition_failure_shutdown_flag;
} Autosequence;


/**
 * Autosequence config and state tracker
 */
extern Autosequence autosequence;

/**
 * Called by set_state in telem.c
 * Modifies global variable STATE
 */
void manual_state_transition(uint8_t next_state);

#endif /* INC_AUTOSEQUENCE_H_ */
