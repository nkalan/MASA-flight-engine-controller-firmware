/*
 * channel_cal_constants.c
 *
 *  Created on: Jun 4, 2021
 *      Author: natha
 */

#include "channel_cal_constants.h"


/* 3.3ADC VREF */
#define VREF                (3.3F)
#define NUM_ADCS            (4U)
#define NUM_ADC_CHANNELS    (16U)
#define MAX_ADC_COUNTS      (2048U)



float adc_counts_to_volts(uint32_t adc_counts, uint8_t adc_num, uint8_t adc_channel) {
	return 0;

}
