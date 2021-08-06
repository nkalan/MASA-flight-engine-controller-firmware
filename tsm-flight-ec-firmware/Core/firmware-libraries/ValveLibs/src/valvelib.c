/*
 * Implementation of wrapper around existing valve functions
 * Elham Islam (eislam@umich.edu)
 * Michigan Aeronautical Science Association
 * Created July 5, 2021
 * Last edited July 5, 2021
 */


#include "../inc/valvelib.h"


void power_valve(Valve *valve){
	for(int i = 0; i < valve->num_channels; i++){
		(*valve->set_valve_func)(valve->valve_channels[i], 1); // set valve channel to high as per valve_channel array
	}
}

void depower_valve(Valve *valve){
	for(int i = 0; i < valve->num_channels; i++){
		(*valve->set_valve_func)(valve->valve_channels[i], 0); // set valve channel to high as per valve_channel array
	}
}

