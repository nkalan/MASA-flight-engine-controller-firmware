/*
 * abort_cases.h
 *
 *  Created on: Jul 5, 2021
 *      Author: natha + ananthas
 * 
 * FUNCTION OBJECTIVE:
 * This function is to automatically detect if an abort should happen based on input parameters
 * 
 * FUNCTION MECHANICS:
 * The function considers a range of possible time values and range of user defined values
 * 
 * The function will determine if (1) the current state is within the time range, and if so, it will then (2) determine if
 * the values considered are within the acceptable user-defined range
 * 
 * If the current state is within the time range but is outside of the user-defined range, then the function returns 1, which
 * can be used to trigger an abort case. In all other case, including when the state is within the user-defined range and when
 * the state is outside of the time range, the function will return 0, which can be used to not trigger an abort case.
 * 
 * Each range is defined by a single treshold value. The threshold can be toggled to be either an upper or lower bound:
 * 		Range:	0 -> Upper Bound
 * 		Range:  Lower Bound -> Infinity
 * 
 * FUNCTION INPUT PARAMETERS:
 * The function has two input parameters, the first is a struct "Threshold_Detection_Config" which is passed in as a pointer
 * and contains several variables. These include a pointer to the variable that is being observed, "observed_var", or in
 * otherwords is the variable you care about that you want to be within a range. The "threshold" varaible defines the 
 * threshold that you are comparing your observed variable to. "time_limit_ms" defines the threshold time value that defines
 * your time range. "tracking_start_time_ms" is the time since the uber-program has started running, this needs to be passed
 * in by the user. "is_upper_timelimit" and "is_upper_threshold" are binary ints used to toggle wether each of the ranges are
 * defined by lower or upper bounds. The second parameter passed into the program is the current time when the function is 
 * called in ms.
 * 
 * 
 * This program has been tested thuroughly through all cases. It was tested on an STM Nucleo Board (F446RE)
 * 
 * 
 */

#ifndef INC_THRESHOLD_DETECTION_H_
#define INC_THRESHOLD_DETECTION_H_

#include <stdint.h>

typedef struct {
	float* observed_var;  // pointer to variable to observe
	float threshold;
	uint32_t time_limit_ms;

	// Set start time and enable at the same time
	uint32_t tracking_start_time_ms;

	// Set a toggle for time (before or after) and wether threshold is upper/lower bound
	uint8_t is_upper_timelimit;
	uint8_t is_upper_threshold;

	// TODO: figure out EXACTLY what it means for a variable to
	// "pass/not pass the threshold within the time limit"
	// because prop probably doesn't know what they actually want
	// this program to do lol

	// TODO: some other config variable for if the threshold is an upper bound
	// or a lower bound
} Threshold_Detection_Config;


/**
 * This function will probably end up being called in the 5ms loop
 * after every time sensors get sampled
 */
uint8_t check_variable_threshold(Threshold_Detection_Config* config, uint32_t current_time_ms);


#endif /* INC_THRESHOLD_DETECTION_H_ */
