/*
 * channel_cal_constants.h
 *
 *  Created on: Jun 4, 2021
 *      Author: natha
 *
 *
 *  File containing ADC channel voltage divider resistor values.
 *  The function converts the adc counts output to (transducer) voltage.
 *
 *  There are 22 ADC channels total on the board, split across several ADCs.
 *  Specify the input (counts) and the sensor channel (0-21),
 *  NOT the raw ADC channel (0-15)
 */

#ifndef INC_CHANNEL_CAL_CONSTANTS_H_
#define INC_CHANNEL_CAL_CONSTANTS_H_

#include "stdint.h"

float adc_counts_to_volts(uint32_t adc_counts, uint8_t adc_num, uint8_t adc_channel);

#endif /* INC_CHANNEL_CAL_CONSTANTS_H_ */
