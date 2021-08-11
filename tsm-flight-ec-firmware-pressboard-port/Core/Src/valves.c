/*
 * valves.c
 *
 *  Created on: Jun 3, 2021
 *      Author: natha
 */

#include "valves.h"
#include "hardware.h"
#include "main.h"

extern uint32_t valve_states;

/*
// Arrays mapping valve channel number to GPIO pins
GPIO_TypeDef* VALVE_GPIO_PORTS[NUM_VALVES] = {
		en_vlv0_GPIO_Port,
		en_vlv1_GPIO_Port,
		en_vlv2_GPIO_Port,
		en_vlv3_GPIO_Port,
		en_vlv4_GPIO_Port,
		en_vlv5_GPIO_Port,
		en_vlv6_GPIO_Port,
		en_vlv7_GPIO_Port,
		en_vlv8_GPIO_Port,
		en_vlv9_GPIO_Port,
		en_vlv10_GPIO_Port,
		en_vlv11_GPIO_Port,
		en_vlv12_GPIO_Port,
		en_vlv13_GPIO_Port
};

uint16_t VALVE_GPIO_PINS[NUM_VALVES] = {
		en_vlv0_Pin,
		en_vlv1_Pin,
		en_vlv2_Pin,
		en_vlv3_Pin,
		en_vlv4_Pin,
		en_vlv5_Pin,
		en_vlv6_Pin,
		en_vlv7_Pin,
		en_vlv8_Pin,
		en_vlv9_Pin,
		en_vlv10_Pin,
		en_vlv11_Pin,
		en_vlv12_Pin,
		en_vlv13_Pin
};
*/

void set_valve_channel(uint32_t vlv_num, uint8_t vlv_state) {
	/*  Flight EC
	// Error checking
	if (vlv_num >= NUM_VALVES) {
		return;
	}

	// Set the correct GPIO pin
	HAL_GPIO_WritePin(VALVE_GPIO_PORTS[vlv_num], VALVE_GPIO_PINS[vlv_num], vlv_state);

	// Update valve_states accordingly
    uint32_t vlv_bit = 1 << vlv_num;
    uint32_t vlv_value = vlv_state << vlv_num;
    valve_states &= ~vlv_bit;  // Clear the previous valve state
    valve_states |= vlv_value;  // Set the new valve state
    */

	// Press board
	setValve(vlv_num, vlv_state);
}
