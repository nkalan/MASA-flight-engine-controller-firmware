/*
 * thermocouples.c
 *
 *  Created on: Oct 3, 2021
 *      Author: natha
 */


#include "MAX31856.h"
#include "main.h"  // for pin mappings
#include "globals.h"  // 'tcs' array lives here

#define EC_NUM_TCS    (12U)

// static global is only visible from this file
static MAX31856_TC_Array thermocoupleArray;

// CS function prototypes
void tc_array_chip_select(uint8_t tc_num);
void tc_array_chip_release();

// Public function definitions
void init_thermocouples(SPI_HandleTypeDef* SPI_bus) { // Need to pass hspi1 as the 'SPI_bus'
	// Define all variables and pointers in the struct in the 'MAX31856.h' file:
	thermocoupleArray.SPI_bus = SPI_bus;
	thermocoupleArray.chip_select = tc_array_chip_select;
	thermocoupleArray.chip_release = tc_array_chip_release;
	thermocoupleArray.num_tcs = EC_NUM_TCS;

	MAX31856_init_thermocouples(thermocoupleArray);

}

void read_thermocouples() {
	// Need to loop through each of the thermocouples by looping through all 12 and calling helper functions
	for(uint8_t tc_index = 0; tc_index < thermocoupleArray.num_tcs; tc_index++){
		tc[tc_index] = MAX31856_read_thermocouple(&thermocoupleArray, tc_index);
	}

}

// Private function definitions

/**
 * Select the correct output on the DEMUX chip
 * to activate only 1 thermocouple chip
 *
 * See schematic or ask for help on details
 * DEMUX part number: Cd74hc154m96
 */
void tc_array_chip_select(uint8_t tc_num) {

}

/**
 * Set the inputs on the DEMUX so that all the thermocouples
 * are inactive
 */
void tc_array_chip_release() {

}