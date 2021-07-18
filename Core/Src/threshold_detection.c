/*
 * abort_cases.c
 *
 *  Created on: Jul 5, 2021
 *      Author: natha + ananthas
 */


#include "threshold_detection.h"



uint8_t check_variable_threshold(Threshold_Detection_Config* config, uint32_t current_time_ms) {


	if(config->is_upper_threshold && config->is_upper_timelimit){

		if(*config->observed_var > config->threshold && (config->tracking_start_time_ms - current_time_ms) < config->time_limit_ms){
			return 1;
		}
	}

	if(!config->is_upper_threshold && config->is_upper_timelimit){
		if(*config->observed_var < config->threshold && (config->tracking_start_time_ms - current_time_ms) < config->time_limit_ms){
			return 1;
		}
	}


	if(config->is_upper_threshold && !config->is_upper_timelimit){
		if(*config->observed_var > config->threshold && (config->tracking_start_time_ms - current_time_ms) > config->time_limit_ms){
			return 1;
		}
	}

	if(!config->is_upper_threshold && !config->is_upper_timelimit){
		if(*config->observed_var < config->threshold && (config->tracking_start_time_ms - current_time_ms) > config->time_limit_ms){
			return 1;
		}
	}
	
	return 0;
}
