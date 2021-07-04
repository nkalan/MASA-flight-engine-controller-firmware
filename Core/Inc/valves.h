/*
 * valves.h
 *
 * There are 14 valves on the flight engine controller.
 * This file includes the function that controls valves and updates
 * valve states, as well as valve GPIO pin mappings.
 *
 *  Created on: Jun 3, 2021
 *      Author: natha
 */

#ifndef INC_VALVES_H_
#define INC_VALVES_H_

#include "stdint.h"

// TODO: should this be defined somewhere else?
#define NUM_VALVES 14

/**
 * Sets the GPIO pin corresponding to the valve channel index specified
 * by vlv_num to the state (high/low) specified by vlv_state.
 *
 * Updates the resulting state in the valve_states global variable.
 *
 * This function will be called by the autosequence, as well as the
 * set_vlv() command for manual control.
 *
 * @param vlv_num A valve channel index to be set, from 0 to 13 inclusive
 * @param vlv_state The state to set the specified valve channel, either high or low (1 or 0)
 */
void set_valve_channel(uint32_t vlv_num, uint8_t vlv_state);

#endif /* INC_VALVES_H_ */
