/*
 * calibrations.c
 *
 *  Created on: Jul 5, 2021
 *      Author: natha
 */


#define ADC_PT_R1 ((float)(1000F));
#define ADC_PT_R2 ((float)(2000F));
#define ADC_PT_VDIV ((float)(ADC_PT_R1 / (ADC_PT_R1 + ADC_PT_R2))
#define ADC_RESOLUTION ((uint16_t)(4096U))

