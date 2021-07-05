/*
 * sensor_voting.c
 *
 *  Created on: Jul 4, 2021
 *      Author: natha
 */


#include "sensor_voting.h"

/**
 * Because of time constraints, this currently just averages the 3 inputs
 * and ignores the config struct.
 *
 * Go ahead and rewrite the function implementation to actually
 * use a voting algorithm.
 */
float sensor_voting_algorithm(float input_A, float input_B, float input_C,
		Voting_Alg_Config* config) {
	return (input_A + input_B + input_C) / 3.0F;
}
