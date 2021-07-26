/*
 * tank_pressure_control.h
 *
 *  Created on: Jun 22, 2021
 *      Author: natha
 */

#ifndef INC_TANK_PRESSURE_CONTROL_H_
#define INC_TANK_PRESSURE_CONTROL_H_

#include <stdint.h>
#include "constants.h"
#include "L6470.h"
//#include "valvelib.h"


/**
 * Each struct instance corresponds to a single propellant tank.
 * It contains active control variables and configuration parameters.
 */
typedef struct {

	/**
	 * Inputs: user must set all these variables on initialization
	 */
	uint8_t tank_enable;   // Enable motor/valve actuations

	uint8_t is_cryogenic;  // Affects initial motor position calculation

	uint8_t control_valve_channel;  // Helium in
	L6470_Motor_IC motor;  // Needle valve

	// These are pointers to detach control algs from
	// sensor reading/voting algs
	float* control_pres;  // tank pressure
	float* COPV_pres;     // for initial motor position
	float* COPV_temp;     // for initial motor position

	// Setpoint
	float target_pres;

	// Thresholds used in Autopress bang bang
	// Pressure above target pressure which to trigger control valve
	// Note: this is a pressure difference, not an absolute pressure
	float bang_bang_low_pres_diff;
	float bang_bang_high_pres_diff;

	// Thresholds used during PID loop. Actuate the tank control valve
	// in addition to the needle valve when control pressure goes a
	// certain range, defined as a percent of the target pressure.
	// Not technically part of the PID calculation, but used in parallel.
	float PID_ctrl_vlv_low_pres_percent;
	float PID_ctrl_vlv_high_pres_percent;

	uint32_t PID_ctrl_loop_period_ms;  // Used in I/D calculations

	float K_p, K_i, K_d;  // Gains


	/*
	 * Outputs: do not directly modify the following variables
	 */
	float Kp_error, Ki_error, Kd_error;  // Error terms for logging
	float PID_error_sum;  // Integral for I term
	float PID_prev_step_error;  // step n-1 for D term
	// ^ TODO: init these as 0


	// TODO: is this needed?
	//volatile double last_error[NUM_TANKS];

	// Updated by tank_PID_pressure_control()
	float motor_setpoint_deg;

	// Updated by check_motor_state()
	float motor_pos_deg;
	float motor_vel_steps_sec;

} TPC_Info;


/**
 * Resets control loop variables and parameters.
 * Call this right before you enter the control loop.
 */
void tank_init_control_loop(TPC_Info* tank);

/**
 * Call this right when you end the control loop for any reason.
 */
void tank_end_control_loop(TPC_Info* tank);

/**
 * Runs a bang bang control loop on the given tank.
 * Should be called at periodic intervals during the AutoPress state.
 */
void tank_autopress_bang_bang(TPC_Info* tank);


/**
 * Main PID control loop that gets called during the engine burn.
 */
void tank_PID_pressure_control(TPC_Info* tank);


/**
 * Bang bang control called in parallel with the PID pressure control loop,
 * but at a different frequency.
 *
 * It accomplishes the same thing as autopress bang bang, but with separate
 * pressure threshold configurations.
 *
 * This function exists for cases when the error is too high for the
 * needle valve to compensate for.
 */
void tank_check_control_valve_threshold(TPC_Info* tank);


/**
 * Before the PID control loop begins, calculate the optimal initial
 * motor position to minimize pressure transients (?).
 */
void tank_startup_init_motor_position(TPC_Info* tank);


/**
 * Reads the absolute position and speed from the motor IC,
 * and updates the variables
 */
void check_motor_state(TPC_Info* tank);

#endif /* INC_TANK_PRESSURE_CONTROL_H_ */
