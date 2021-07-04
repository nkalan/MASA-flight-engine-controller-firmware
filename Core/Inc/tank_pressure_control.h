/*
 * tank_pressure_control.h
 *
 *  Created on: Jun 22, 2021
 *      Author: natha
 */

#ifndef INC_TANK_PRESSURE_CONTROL_H_
#define INC_TANK_PRESSURE_CONTROL_H_

#include <stdint.h>

//#include "L6740.h"
//#include valves library

// TODO: is this needed in this file?
#define LOX_TANK   (0)
#define FUEL_TANK  (1)

typedef struct {
	uint8_t tank_enable;

	// Control valve
	// L6470_Motor_IC

	// This is a pointer, to detach control algs from sensor voting algs
	float* control_pres;  // tank pressure

	// Setpoint
	float PID_target_pres;

	// Thresholds used during PID loop. Actuate the tank control valve
	// in addition to the needle valve when control pressure goes
	// outside a certain range.
	float PID_ctrl_vlv_low_pres_thrsh;
	float PID_ctrl_vlv_high_pres_thrsh;

	float bang_bang_low_percent_threshold;
	float bang_bang_high_percent_threshold;

	float pass_low_tol[2] = { -50, -50 };
	float pass_high_tol[2] = { 50, 50 };
} TPC_Info;


void tank_autopress_bang_bang(TPC_Info* tank);

void tank_PID_pressure_control(TPC_Info* tank);

void tank_check_control_valve_threshold(TPC_Info* tank);


#endif /* INC_TANK_PRESSURE_CONTROL_H_ */
