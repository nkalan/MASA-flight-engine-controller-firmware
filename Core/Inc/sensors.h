/*
 * sensors.h
 *
 *  Created on: Jun 5, 2021
 *      Author: natha
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_


void init_thermocouples();


void read_thermocouples();


void init_adcs();


void read_adc_counts();


void convert_adc_counts();


#endif /* INC_SENSORS_H_ */
