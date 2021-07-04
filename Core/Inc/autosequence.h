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


#define BIGBERTHA
//#define PT163

/**
 * Sets valve and test timings to their default values
 * All numbers are in milliseconds
 */
void init_autosequence_timings();

/**
 * Called by set_state in telem.c
 * Modifies global variable STATE
 */
void manual_state_transition(uint8_t next_state);

#endif /* INC_AUTOSEQUENCE_H_ */
