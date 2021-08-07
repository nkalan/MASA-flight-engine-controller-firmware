/*
 * sensors.h
 *
 *  Created on: Jun 5, 2021
 *      Author: natha
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "MAX31856.h"


extern MAX31856_TC_Array thermocouples;

void tc_mux_chip_select(uint8_t tc_index);

void tc_mux_chip_release();

/**
 * Initialize all thermocouples
 */
void init_thermocouples();


/**
 * Read all thermocouples
 */
void read_thermocouples();


/**
 * Initialize ADCs
 */
void init_adcs();


/**
 * Read counts from all ADCs
 */
void read_adc_counts();


/**
 * Convert ADC counts to real values
 */
void convert_adc_counts();


/**
 * Run sensor voting algorithms for LOX tank pressure,
 * fuel tank pressure, and COPV pressure
 */
void resolve_redundant_sensors();


#endif /* INC_SENSORS_H_ */
