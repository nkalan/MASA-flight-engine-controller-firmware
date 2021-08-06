/*
 * Header file for wrapper around existing valve functions
 *
 * Elham Islam (eislam@umich.edu)
 * Michigan Aeronautical Science Association
 * Created July 5, 2021
 * Last edited July 5, 2021
 */

#ifndef INC_VALVELIB_H_ // Begin header include protection
#define INC_VALVELIB_H_

#define VALVELIB_MAX_CHANNELS (5)

#include "stdint.h"

typedef struct {
	uint8_t num_channels;
	uint8_t valve_channels[VALVELIB_MAX_CHANNELS]; // array containing states of each valve
	void (*set_valve_func)(uint32_t, uint8_t); // existing function used to set valve states

} Valve;

void power_valve(Valve *valve); //set valve channels high

void depower_valve(Valve *valve); //set valve channels low


#endif /* INC_VALVELIB_H_ */
