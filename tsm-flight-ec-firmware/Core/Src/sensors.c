/*
 * sensors.c
 *
 *  Created on: Jun 5, 2021
 *      Author: natha
 */


#include "sensors.h"
#include "sensor_voting.h"
#include "constants.h"
#include "globals.h"
#include "MAX11128.h"
#include "main.h"  // for GPIO mappings

#define NUM_ADCS            (4U)
#define CHANNELS_PER_ADC   (16U)

/* ADC0 */
#define IVLV0_ADC_NUM       (0U)
#define EVLV0_ADC_NUM       (0U)
#define IVLV1_ADC_NUM       (0U)
#define EVLV1_ADC_NUM       (0U)
#define IVLV2_ADC_NUM       (0U)
#define EVLV2_ADC_NUM       (0U)
#define IVLV3_ADC_NUM       (0U)
#define EVLV3_ADC_NUM       (0U)
#define IVLV4_ADC_NUM       (0U)
#define EVLV4_ADC_NUM       (0U)
#define IVLV5_ADC_NUM       (0U)
#define EVLV5_ADC_NUM       (0U)
#define IVLV6_ADC_NUM       (0U)
#define EVLV6_ADC_NUM       (0U)
#define IVLV7_ADC_NUM       (0U)
#define EVLV7_ADC_NUM       (0U)

#define IVLV0_ADC_CH        (0U)
#define EVLV0_ADC_CH        (1U)
#define IVLV1_ADC_CH        (2U)
#define EVLV1_ADC_CH        (3U)
#define IVLV2_ADC_CH        (4U)
#define EVLV2_ADC_CH        (5U)
#define IVLV3_ADC_CH        (6U)
#define EVLV3_ADC_CH        (7U)
#define IVLV4_ADC_CH        (8U)
#define EVLV4_ADC_CH        (9U)
#define IVLV5_ADC_CH       (10U)
#define EVLV5_ADC_CH       (11U)
#define IVLV6_ADC_CH       (12U)
#define EVLV6_ADC_CH       (13U)
#define IVLV7_ADC_CH       (14U)
#define EVLV7_ADC_CH       (15U)

/* ADC1 */
#define IVLV8_ADC_NUM       (1U)
#define EVLV8_ADC_NUM       (1U)
#define IVLV9_ADC_NUM       (1U)
#define EVLV9_ADC_NUM       (1U)
#define IVLV10_ADC_NUM      (1U)
#define EVLV10_ADC_NUM      (1U)
#define IVLV11_ADC_NUM      (1U)
#define EVLV11_ADC_NUM      (1U)
#define IVLV12_ADC_NUM      (1U)
#define EVLV12_ADC_NUM      (1U)
#define IVLV13_ADC_NUM      (1U)
#define EVLV13_ADC_NUM      (1U)

#define IVLV8_ADC_CH        (4U) // 0-3 are disconnected
#define EVLV8_ADC_CH        (5U)
#define IVLV9_ADC_CH        (6U)
#define EVLV9_ADC_CH        (7U)
#define IVLV10_ADC_CH       (8U)
#define EVLV10_ADC_CH       (9U)
#define IVLV11_ADC_CH      (10U)
#define EVLV11_ADC_CH      (11U)
#define IVLV12_ADC_CH      (12U)
#define EVLV12_ADC_CH      (13U)
#define IVLV13_ADC_CH      (14U)
#define EVLV13_ADC_CH      (15U)

/* ADC2 */
#define EBATT_ADC_NUM       (2U)
#define IBUS_ADC_NUM        (2U)
#define E5V_ADC_NUM         (2U)
#define E3V3_ADC_NUM        (2U)
#define I3V3_ADC_NUM        (2U)
#define I5V_ADC_NUM         (2U)
#define TBRD0_ADC_NUM       (2U)
#define TBRD1_ADC_NUM       (2U)
#define TBRD2_ADC_NUM       (2U)
#define PT0_ADC_NUM         (2U)
#define PT1_ADC_NUM         (2U)
#define PT2_ADC_NUM         (2U)
#define PT3_ADC_NUM         (2U)
#define PT4_ADC_NUM         (2U)
#define PT5_ADC_NUM         (2U)
#define PT6_ADC_NUM         (2U)

#define EBATT_ADC_CH        (0U)
#define IBUS_ADC_CH         (1U)
#define E5V_ADC_CH          (2U)
#define E3V3_ADC_CH         (3U)
#define I3V3_ADC_CH         (4U)
#define I5V_ADC_CH          (5U)
#define TBRD0_ADC_CH        (6U)
#define TBRD1_ADC_CH        (7U)
#define TBRD2_ADC_CH        (8U)
#define PT0_ADC_CH          (9U)
#define PT1_ADC_CH         (10U)
#define PT2_ADC_CH         (11U)
#define PT3_ADC_CH         (12U)
#define PT4_ADC_CH         (13U)
#define PT5_ADC_CH         (14U)
#define PT6_ADC_CH         (15U)

/* ADC3 */
#define PT7_ADC_NUM         (3U)
#define PT8_ADC_NUM         (3U)
#define PT9_ADC_NUM         (3U)
#define PT10_ADC_NUM        (3U)
#define PT11_ADC_NUM        (3U)
#define PT12_ADC_NUM        (3U)
#define PT13_ADC_NUM        (3U)
#define PT14_ADC_NUM        (3U)
#define PT15_ADC_NUM        (3U)
#define PT16_ADC_NUM        (3U)
#define PT17_ADC_NUM        (3U)
#define PT18_ADC_NUM        (3U)
#define PT19_ADC_NUM        (3U)
#define PT20_ADC_NUM        (3U)
#define POT0_ADC_NUM        (3U)
#define POT0_ADC_NUM        (3U)

#define PT7_ADC_CH          (0U)
#define PT8_ADC_CH          (1U)
#define PT9_ADC_CH          (2U)
#define PT10_ADC_CH         (3U)
#define PT11_ADC_CH         (4U)
#define PT12_ADC_CH         (5U)
#define PT13_ADC_CH         (6U)
#define PT14_ADC_CH         (7U)
#define PT15_ADC_CH         (8U)
#define PT16_ADC_CH         (9U)
#define PT17_ADC_CH        (10U)
#define PT18_ADC_CH        (11U)
#define PT19_ADC_CH        (12U)
#define PT20_ADC_CH        (13U)
#define POT0_ADC_CH        (14U)
#define POT1_ADC_CH        (15U)


/**
 * Defining thermocouple interface struct
 */
MAX31856_TC_Array thermocouples;

/**
 * Defining ADC interface struct and rx arrays
 */
GPIO_MAX11128_Pinfo ADC_structs[NUM_ADCS];
uint16_t adc_values[NUM_ADCS][CHANNELS_PER_ADC] = {0};


/**
 * Function passed to the TC Array library as the chip select.
 * Sets 4 GPIO pins to get the correct MUX CS output.
 */
void tc_mux_chip_select(uint8_t tc_index) {

	// Extract bits
	uint8_t tc_mux_0 = tc_index & 0x01;
	uint8_t tc_mux_1 = tc_index & 0x02;
	uint8_t tc_mux_2 = tc_index & 0x04;
	uint8_t tc_mux_3 = tc_index & 0x08;

	// Set mux select bits
	HAL_GPIO_WritePin(TC_MUX_A0_GPIO_Port, TC_MUX_A0_Pin, tc_mux_0);
	HAL_GPIO_WritePin(TC_MUX_A1_GPIO_Port, TC_MUX_A1_Pin, tc_mux_1);
	HAL_GPIO_WritePin(TC_MUX_A2_GPIO_Port, TC_MUX_A2_Pin, tc_mux_2);
	HAL_GPIO_WritePin(TC_MUX_A3_GPIO_Port, TC_MUX_A3_Pin, tc_mux_3);

	// Active low mux enable
	HAL_GPIO_WritePin(TC_MUX_EN_GPIO_Port, TC_MUX_EN_Pin, 0);
}

/**
 * Function passed to the TC Array library as the chip release.
 */
void tc_mux_chip_release() {
	// Active low, will set all mux outputs to 1
	HAL_GPIO_WritePin(TC_MUX_EN_GPIO_Port, TC_MUX_EN_Pin, 1);
}

/**
 * Set thermocouple array according to MAX31856 library
 */
void init_thermocouples() {
	thermocouples.num_tcs = NUM_TCS;
	thermocouples.SPI_bus = &SPI_TC;
	thermocouples.chip_select = tc_mux_chip_select;
	thermocouples.chip_release = tc_mux_chip_release;

	MAX31856_init_thermocouples(&thermocouples);
}


void read_thermocouples() {
	for (uint8_t i = 0; i < NUM_TCS; i++) {
		tc[i] = MAX31856_read_thermocouple(&thermocouples, i);
	}
}


void init_adcs() {
	// Pin configurations
	ADC_structs[0].MAX11128_CS_PORT = ADC0_CS_GPIO_Port;
	ADC_structs[0].MAX11128_EOC_PORT = ADC0_EOC_GPIO_Port;
	ADC_structs[0].MAX11128_CS_PORT = ADC0_CS_Pin;
	ADC_structs[0].MAX11128_EOC_PORT = ADC0_EOC_Pin;
	ADC_structs[0].HARDWARE_CONFIGURATION = EOC_ONLY;

	ADC_structs[1].MAX11128_CS_PORT = ADC1_CS_GPIO_Port;
	ADC_structs[1].MAX11128_EOC_PORT = ADC1_EOC_GPIO_Port;
	ADC_structs[1].MAX11128_CS_PORT = ADC1_CS_Pin;
	ADC_structs[1].MAX11128_EOC_PORT = ADC1_EOC_Pin;
	ADC_structs[1].HARDWARE_CONFIGURATION = EOC_ONLY;

	ADC_structs[2].MAX11128_CS_PORT = ADC2_CS_GPIO_Port;
	ADC_structs[2].MAX11128_EOC_PORT = ADC2_EOC_GPIO_Port;
	ADC_structs[2].MAX11128_CS_PORT = ADC2_CS_Pin;
	ADC_structs[2].MAX11128_EOC_PORT = ADC2_EOC_Pin;
	ADC_structs[2].HARDWARE_CONFIGURATION = EOC_ONLY;

	ADC_structs[3].MAX11128_CS_PORT = ADC3_CS_GPIO_Port;
	ADC_structs[3].MAX11128_EOC_PORT = ADC3_EOC_GPIO_Port;
	ADC_structs[3].MAX11128_CS_PORT = ADC3_CS_Pin;
	ADC_structs[3].MAX11128_EOC_PORT = ADC3_EOC_Pin;
	ADC_structs[3].HARDWARE_CONFIGURATION = EOC_ONLY;

	// Initialization commands
	init_adc(&SPI_ADC, &ADC_structs[0]);
	init_adc(&SPI_ADC, &ADC_structs[1]);
	init_adc(&SPI_ADC, &ADC_structs[2]);
	init_adc(&SPI_ADC, &ADC_structs[3]);
}


void read_adc_counts() {
	read_adc(&SPI_ADC, &ADC_structs[0], adc_values[0]);
	read_adc(&SPI_ADC, &ADC_structs[1], adc_values[1]);
	read_adc(&SPI_ADC, &ADC_structs[2], adc_values[2]);
	read_adc(&SPI_ADC, &ADC_structs[3], adc_values[3]);
}


void convert_adc_counts() {

}


void resolve_redundant_sensors() {
	// TODO: fill out configs when they're ready
	Voting_Alg_Config lox_voting_config;
	Voting_Alg_Config fuel_voting_config;
	Voting_Alg_Config copv_voting_config;

	lox_control_pressure = sensor_voting_algorithm(pressure[LOX_TANK_A_PRES_CH],
			pressure[LOX_TANK_B_PRES_CH], pressure[LOX_TANK_C_PRES_CH],
			&lox_voting_config);
	fuel_control_pressure = sensor_voting_algorithm(pressure[FUEL_TANK_A_PRES_CH],
			pressure[FUEL_TANK_B_PRES_CH], pressure[FUEL_TANK_C_PRES_CH],
			&fuel_voting_config);
	copv_control_pressure = sensor_voting_algorithm(pressure[COPV_A_PRES_CH],
			pressure[COPV_B_PRES_CH], pressure[COPV_C_PRES_CH],
			&copv_voting_config);
}

