/*
 * adcs.c
 *
 *  Created on: Oct 3, 2021
 *      Author: natha
 */

#include "adcs.h"
#include "main.h"
#include "globals.h"
#include "MAX11128.h"

#define EC_NUM_ADCS             (4U)
#define ADC_NUM_CHANNELS       (16U)

// Static global is only visible in this .c file
static GPIO_MAX11128Pinfo adc_structs[EC_NUM_ADCS];

// Private function prototype
void convert_adc_counts();

// Public function definitions

void init_adcs() {

}

// Call relevant MAX11128 functions to fill an array of
// unsigned 16bit integers
// There are 4 ADCs, indexed 0-3
void read_adcs() {
	uint16_t adc_counts[EC_NUM_ADCS][ADC_NUM_CHANNELS];

}

// Private function definitions

/**
 * Apply calibrations to the raw ADC counts and store the results in the
 * telemetry packet variables in globals.h
 * This step requires knowledge of the electrical characteristics
 * of the sensors and circuitry
 */
void convert_adc_counts(uint16_t** adc_counts) {

}
