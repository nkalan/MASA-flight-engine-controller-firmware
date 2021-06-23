/*
 * motors.h
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "L6470.h"  // stepper motor IC

#define NUM_STEPPER_MOTORS (2)

void init_motor(uint8_t mtr_id);

void set_motor_pos(uint8_t motor_num, float deg);

// Also see set_stepper_direction(), move_stepper_degrees(), set_stepper_pos(),
// set_stepper_zero(), and set_stepper_speed() in telem.c

#endif /* INC_MOTORS_H_ */
