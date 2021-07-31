/*
 * sensor_voting.c
 *
 *  Created on: Jul 4, 2021
 *      Author: Elham Islam
 */


#include "sensor_voting.h"
#include "status_flags.h"

#define MEOP_LOX (340)
#define MEOP_COPV (1400)
#define MEOP_FUEL (340)

/**
 * Because of time constraints, this currently just averages the 3 inputs
 * and ignores the config struct.
 *
 * Go ahead and rewrite the function implementation to actually
 * use a voting algorithm.
 */
float sensor_voting_algorithm(float input_A, float input_B, float input_C, char tank) {


	// QUESTIONS: should we just pass in the meop as a float or pass the type of tank and then it references a constant?
	// Are still passing in the invalid input? Yeah right
	// Will the flag count change?

	static int sensor_fail_count = 0;
	float valid_input_1, valid_input_2, threshold, flag;
	float threshold_factor = 0.05

	// Determine threshold
	switch(tank){
		case 'L': // LOX
			threshold = MEOP_LOX*threshold_factor;
		case 'F': // Fuel
			threshold = MEOP_FUEL*threshold_factor;
		case 'C': //COPV
			threshold = MEOP_COPV*threshold_factor;
	}

	// Check if input A is invalid
	if ((abs(input_A - input_B) > threshold) && (abs(input_A - input_C) > threshold) && (abs(input_B - input_C) < threshold))
	{
		valid_input_1 = input_B;
		valid_input_2 = input_C;

		switch(tank){
				case 'L': set_status_flag(EC_FLAG_LOX_PT_A_FAIL);
				case 'F': set_status_flag(EC_FLAG_FUEL_PT_A_FAIL);
				case 'C': set_status_flag(EC_FLAG_COPV_PT_A_FAIL);
			}
	} // Checks if input B is invalid
	else if((abs(input_B - input_C) > threshold) && (abs(input_B - input_A) > threshold) && (abs(input_A - input_C) < threshold))
	{
		valid_input_1 = input_A;
		valid_input_2 = input_C;

		switch(tank){
				case 'L': set_status_flag(EC_FLAG_LOX_PT_B_FAIL);
				case 'F': set_status_flag(EC_FLAG_FUEL_PT_B_FAIL);
				case 'C': set_status_flag(EC_FLAG_COPV_PT_B_FAIL);
			}
	}
	 // Checks if input C is invalid
	else if((abs(input_C - input_A) > threshold) && (abs(input_C - input_B) > threshold) && (abs(input_A - input_B) < threshold))
	{
		valid_input_1 = input_A;
		valid_input_2 = input_B;

		switch(tank){
				case 'L': set_status_flag(EC_FLAG_LOX_PT_C_FAIL);
				case 'F': set_status_flag(EC_FLAG_FUEL_PT_C_FAIL);
				case 'C': set_status_flag(EC_FLAG_COPV_PT_C_FAIL);
			}

	}


	// return average of valid inputs
	if (sensor_fail_count == 0) {
		return (input_A + input_B + input_C) / 3.0F;
	}
	else if(sensor_fail_count == 1){
		return (valid_input_1 + valid_input_1)/ 2.0F;
	}
	/*else{
		return("we fucked lol");
	}
	*/
}
