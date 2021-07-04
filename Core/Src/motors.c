/*
 * motors.c
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#include "motors.h"

void set_motor_pos(uint8_t motor_num, float deg) {
	// Error checking
	if (motor_num >= NUM_MOTORS) {
		return;
	}

	/*
	manual_stepper_pos_override[motor_num] = 1;
	targetPos[motor_num] = position; // position converted form deg to steps
	curDir[motor_num] = (curPos[motor_num] < targetPos[motor_num]) ? 1 : -1; // CCW facing the motor
	mtr_set[motor_num] = position; // save new motor position setpoint
	*/
}
