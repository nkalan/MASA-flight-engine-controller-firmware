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


// TODO: not sure all of these options are needed
typedef enum {
	Water,    // Press tests, coldflows
	LN2,      // Press tests, coldflows
	Ethanol,  // PT163 hotfires
	RP1,      // BB hotfires
	LOX       // PT163/BB hotfires
} Fluid_Type;

/**
 * Each struct instance corresponds to a single propellant tank.
 * It contains active control variables and configuration parameters.
 */
typedef struct {
	uint8_t tank_enable;  // Enables motor/control valve actuation

	uint8_t is_cryogenic;  // Affects initial motor position calculation

	// TODO: Control valve struct pointer
	// TODO: L6470_Motor_IC pointer

	// This is a pointer to detach PID from sensor reading/voting algs
	float* control_pres;  // tank pressure

	// Setpoint
	float target_pres;

	// Thresholds used in Autopress bang bang
	// Absolute pressure below which to trigger control valve
	// Note: this is an absolute pressure, unlike the press board code.
	float bang_bang_low_pres_thrshd;
	float bang_bang_high_pres_thrshd;

	// Thresholds used during PID loop. Actuate the tank control valve
	// in addition to the needle valve when control pressure goes
	// outside a certain range.
	// Not technically part of the PID calculation, but used in parallel
	float PID_ctrl_vlv_low_pres_thrshd;
	float PID_ctrl_vlv_high_pres_thrshd;

	uint32_t PID_start_time;  // Used in I calculations TODO: not sure if needed?
	uint32_t PID_ctrl_loop_period;  // Used in I/D calculations

	float K_p, K_i, K_d;  // Gains
	float PID_error_sum;  // Integral for I term
	float PID_prev_step_error;  // step n-1 for D term
	// ^ TODO: init these as 0


	// TODO: figure out if a timer needs to be used here, or if
	// hardcoding dt is good enough


	//---------------------------------------------------------------------------------------------
	//PID variables
	//double errSum[NUM_TANKS] = {0};
	volatile double last_error[NUM_TANKS];
	//double kp_term, ki_term, kd_term;  local, doesn't need to be in struct
	double /*timeChange,*/ error;
	//float control_loop_period;

} TPC_Info;

/**
 * Resets control loop variables and parameters. Call this during
 * program initialization, and before each time you enter the control loop.
 */
void tank_init_control_loop(TPC_Info* tank);


/**
 * Runs a bang bang control loop on the given tank.
 * Should be called at periodic intervals during the AutoPress state.
 *
 * Required tank variables to be configured beforehand:
 * bang_bang_low_pres_thrshd
 * bang_bang_high_pres_thrshd
 * control_pres
 * Control Valve
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
void tank_startup_init_motor_position(TPC_Info* tank, float COPV_pres,
		float COPV_temp);

#endif /* INC_TANK_PRESSURE_CONTROL_H_ */
