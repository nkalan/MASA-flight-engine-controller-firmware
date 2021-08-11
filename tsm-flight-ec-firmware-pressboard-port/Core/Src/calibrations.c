/*
 * calibrations.c
 *
 *  Created on: Jul 30, 2021
 *      Author: natha
 */


#include "calibrations.h"
#include "constants.h"

#define ADC_COUNTS_TO_VOLTS (3.3/4096)

//#define PT_4WIRE_5V_VDIV_GAIN  (0.6565614618)   // R1=5.6k, R2=3k,  RG=56k
//#define PT_3WIRE_5V_VDIV_GAIN  (0.6511627907)   // R1=3k,   R2=5.6k
//#define PT_3WIRE_12V_VDIV_GAIN  (0.2619047619)  // R1=6.2k, R2=2.2k

//#define EVLV_VDIV_GAIN   (0.2481203007)  // R1=10k, R2=3.3k
//#define E3V3_VDIV_GAIN   (0.5)           // R1=1k, R2=1k
//#define E5V_VDIV_GAIN    (0.5)           // R1=1k, R2=1k
//#define VBATT_VDIV_GAIN  (0.2481203007)  // R1=10k, R2=3.3k
//#define

/* Calibration Constants */
// TODO: update cals, these are taken from the Press board, which is taken from ECS3
// TODO: are these fine?
#define IBATT_CAL           (0.01611721)   // checked
#define EBATT_CAL           (0.00324707)   // checked
#define EVLV_CAL            (0.00324707)   // checked
#define IVLV_CAL            (0.00322265)   // checked
#define IMTR_CAL            (0.00322265)
#define E5V_CAL             (0.00161132)   // checked
#define E3V_CAL             (0.00161132)   // checked
#define I5V_CAL             (1.00000000)   // checked
#define I3V_CAL             (1.00000000)   // checked

#define EPOT_DMAX           (3610)  // degs
#define EPOT_DMIN           (0)     // degs
#define EPOT_RES_LEAD       (0.9)   // ohms
#define EPOT_RES_POT        (1000)  // ohms
#define EPOT_RES_POT_MIN    (1)     // ohms
#define EPOT_MAX_COUNTS     (4096)  // counts
#define EPOT_CHAR_OFFSET    (0.0F)  // from testing characterization
#define EPOT_CHAR_SLOPE     (1.0F)  // from testing characterization

#define POT_CAL_SLOPE  ((EPOT_DMAX - EPOT_DMIN)*(EPOT_RES_POT+2*EPOT_RES_LEAD)/(EPOT_MAX_COUNTS*(EPOT_RES_POT - 2.0*EPOT_RES_POT_MIN)))
#define POT_CAL_OFFSET (((EPOT_MAX_COUNTS*(EPOT_RES_POT_MIN + EPOT_RES_LEAD))/(EPOT_RES_POT + 2.0*EPOT_RES_LEAD)))

#define PT_3WIRE_5V_VDIV_GAIN   (0.6511627907)   // R1=3k,   R2=5.6k
#define PT_3WIRE_12V_VDIV_GAIN  (0.2608695652)   // R1=6.8k, R2=2.4k


float pt_ambients[NUM_PTS] = { 0 };
float pot_ambients[NUM_POTS] = { 0 };

float pt_cal_lower_voltage[NUM_PTS] = { 0 };
float pt_cal_upper_voltage[NUM_PTS] = { 0 };
float pt_cal_upper_pressure[NUM_PTS] = { 0 };


float pot_counts_to_deg(uint8_t pot_num, uint16_t counts) {
    // negative sign to align motor pos direction with pot pos direction
    return -EPOT_CHAR_SLOPE*((POT_CAL_SLOPE * (counts - POT_CAL_OFFSET))
    		- pot_ambients[pot_num] - EPOT_CHAR_OFFSET);
}

/**
 * Use the channel-specific calibrations to convert voltage back to pressure
 */
float pt_counts_to_psi(uint8_t pt_num, uint16_t pt_counts) {

	if (pt_num < NUM_PTS) {
		// Convert adc counts to ducer volts
		float pt_volts;
		if (pt_num == 5) {  // Channel 5 is 12V
			pt_volts = pt_counts*ADC_COUNTS_TO_VOLTS/PT_3WIRE_12V_VDIV_GAIN;
		}
		else {
			pt_volts = pt_counts*ADC_COUNTS_TO_VOLTS/PT_3WIRE_5V_VDIV_GAIN;
		}

		// Convert ducer volts to psi
		if ( pt_cal_upper_voltage[pt_num] - pt_cal_lower_voltage[pt_num] != 0) {  // protect from divide by 0
			return ((pt_volts - pt_cal_lower_voltage[pt_num]) * pt_cal_upper_pressure[pt_num]
				 / (pt_cal_upper_voltage[pt_num] - pt_cal_lower_voltage[pt_num]));
		}
	}
	return -1;
}
