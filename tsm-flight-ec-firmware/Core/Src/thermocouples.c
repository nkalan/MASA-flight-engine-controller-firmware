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
static MAX31856_TC_Array;

// CS function prototypes
void tc_array_chip_select(uint8_t tc_num);
void tc_array_chip_release();

// Public function definitions
void init_thermocouples() {

}

void read_thermocouples() {

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
