/*
 * motors.h
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "L6470.h"  // stepper motor IC


void init_mtr(uint8_t mtr_id);

void set_mtr_pos(uint8_t mtr_id, uint16_t mtr_pos);

void set_mtr_spd(uint8_t mtr_id, uint16_t mtr_spd);

#endif /* INC_MOTORS_H_ */
