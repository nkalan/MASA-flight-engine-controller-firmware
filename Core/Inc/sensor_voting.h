/*
 * sensor_voting.h
 *
 *  Created on: Jul 4, 2021
 *      Author: natha
 */

#ifndef INC_SENSOR_VOTING_H_
#define INC_SENSOR_VOTING_H_

#include "status_flags.h"

#define MEOP_LOX (340)
#define MEOP_COPV (1400)
#define MEOP_FUEL (340)

typedef struct {

	// Voting algorithm parameters are left as a struct because
	// of unclear requirements
} Voting_Alg_Config;


/**
 * Accepts 3 floats as inputs, as well as a configuration struct.
 * Outputs 1 float, which "combines" the 3 inputs.
 *
 * The 3 floats are supposed to be from sensors that measure the same thing,
 * so this algorithm can detect and reject up to 1 faulty sensor, and
 * combine the valid sensor readings to output a more accurate measurement.
 */
float sensor_voting_algorithm(float input_A, float input_B, float input_C,
		float meop);




#endif /* INC_SENSOR_VOTING_H_ */
