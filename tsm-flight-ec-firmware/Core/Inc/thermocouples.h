/*
 * thermocouples.h
 *
 *  Created on: Oct 3, 2021
 *      Author: natha
 */

#ifndef INC_THERMOCOUPLES_H_
#define INC_THERMOCOUPLES_H_

/**
 * Set the pinout of the MAX31856 thermocouple array struct
 * and call its initialization function
 *
 * See MAX31856.h README for implementation details
 */
void init_thermocouples();

/**
 * Call this function in a periodic loop to read temperature
 * data from the thermocouples
 *
 * Implement the function calls in the MAX31856 library
 *
 * Modifies the 'tcs' array in globals.h, fills it with
 * temperature in Kelvin
 */
void read_thermocouples();

#endif /* INC_THERMOCOUPLES_H_ */
