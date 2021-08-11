/*
 * calibraitons.h
 *
 *  Created on: Jul 30, 2021
 *      Author: natha
 */

#ifndef INC_CALIBRATIONS_H_
#define INC_CALIBRATIONS_H_

#include <stdint.h>
#include "constants.h"


extern float pt_ambients[NUM_PTS];
extern float pot_ambients[NUM_POTS];

extern float pt_cal_lower_voltage[NUM_PTS];
extern float pt_cal_upper_voltage[NUM_PTS];
extern float pt_cal_upper_pressure[NUM_PTS];

// TODO: add all other

//float pot_counts_to_deg(uint8_t pot_num, uint16_t counts);

float pt_counts_to_psi(uint8_t pt_num, uint16_t pt_volts);

#endif /* INC_CALIBRATIONS_H_ */
