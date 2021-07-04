/*
 * motors.h
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdint.h>

//#include "L6470.h"  // stepper motor IC

#define NUM_MOTORS (2)


// Grabbing global variables from globals.c
extern float mtr_pos[NUM_MOTORS];
extern int16_t mtr_vel[NUM_MOTORS];  // Not sure if needed
extern float mtr_set[NUM_MOTORS];
extern float mtr_ki[NUM_MOTORS];
extern float mtr_kd[NUM_MOTORS];
extern float mtr_kp[NUM_MOTORS];

void init_motor(uint8_t mtr_id);

void set_motor_pos(uint8_t motor_num, float deg);

// Also see set_stepper_direction(), move_stepper_degrees(), set_stepper_pos(),
// set_stepper_zero(), and set_stepper_speed() in telem.c

#endif /* INC_MOTORS_H_ */
