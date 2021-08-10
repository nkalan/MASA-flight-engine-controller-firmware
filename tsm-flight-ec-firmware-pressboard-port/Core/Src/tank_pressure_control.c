/*
 * tank_pressure_control.c
 *
 *  Created on: Jun 22, 2021
 *      Author: natha
 */


#include "tank_pressure_control.h"
#include "hardware.h"
#include "constants.h"
#include "valves.h"
#include "math.h"

extern float init_motor_pos_deg_correction_factor;  // from globals


/**
 * Small wrapper around motor actuation, to allow specific
 * tanks to be disabled. Calculations are always run, but actuations
 * are stopped when tank_enable is false.
 */
/*
void actuate_tank_motor_pos(TPC_Info* tank, float motor_pos) {
	if (tank->tank_enable) {
		// actuate motor to pos
	}
}
*/

/**
 * Small wrapper around control valve actuation, to allow specific
 * tanks to be disabled. Calculations are always run, but actuations
 * are stopped when tank_enable is false.
 */
void actuate_tank_control_valve(TPC_Info* tank, uint8_t state) {
	if (tank->tank_enable) {
		set_valve_channel(tank->control_valve_channel, state);
	}
}

/**
 * Call this right before entering the control loop
 */
void tank_init_control_loop(TPC_Info* tank) {
	tank->Kp_error = 0,
	tank->Ki_error = 0;
	tank->Kd_error = 0;
	tank->PID_error_sum = 0;
	tank->PID_prev_step_error = tank->target_pres - *(tank->control_pres);
}


void tank_autopress_bang_bang(TPC_Info* tank) {
	if (*(tank->control_pres) < (tank->target_pres
			+ tank->bang_bang_low_pres_diff)) {
		actuate_tank_control_valve(tank, 1);
		// TODO: double check normally open normally closed
	}
	else if (*(tank->control_pres) > (tank->target_pres
			+ tank->bang_bang_high_pres_diff)) {
		actuate_tank_control_valve(tank, 0);
	}
}


void tank_PID_pressure_control(TPC_Info* tank) {
	float dt = (tank->PID_ctrl_loop_period_ms)/1000.0;
    float max_motor_delta = maxPos - curPos[tank->motor_num]; //distance from maxPos

	// The missile knows where it is
	float error = tank->target_pres - *(tank->control_pres);  // P
	float next_error_sum = tank->PID_error_sum + error*dt;    // I
	float slope = (error - tank->PID_prev_step_error) / dt;   // D
	tank->PID_prev_step_error = error;  // store for next D calculation

	// Apply gains
	float Kp_term = tank->K_p * error;
	float Ki_term = tank->K_i * tank->PID_error_sum;
	float Kd_term = tank->K_d * slope;
	float PID_total_output = Kp_term + Ki_term + Kd_term;

	// Limit output
	float motor_delta;
	if (PID_total_output < -curPos[tank->motor_num]) {  // Lower bound
		motor_delta = -curPos[tank->motor_num];
	}
	else if (PID_total_output > max_motor_delta) {  // Upper bound
		motor_delta = max_motor_delta;
	}
	else {
		motor_delta = PID_total_output;
		// Update sum IFF output doesn't saturate, to prevent integrator windup
		tank->PID_error_sum = next_error_sum;
	}

	// Actuate motor to new position
	//actuate_tank_motor_pos(tank, motor_pos + motor_delta);  Not yet lol
    targetPos[tank->motor_num] = curPos[tank->motor_num] + motor_delta;
    curDir[tank->motor_num] = (curPos[tank->motor_num] < targetPos[tank->motor_num]) ? 1 : -1;

	// Log data
    mtr_set[tank->motor_num] = targetPos[tank->motor_num];
    tank->Kp_error = Kp_term;
    tank->Ki_error = Ki_term;
    tank->Kd_error = Kd_term;
}


// Almost identical to autopress bang bang but it runs in parallel
// with the PID control loop and has different thresholds.
void tank_check_control_valve_threshold(TPC_Info* tank) {
    if (*(tank->control_pres) < (tank->PID_ctrl_vlv_low_pres)) {
    	actuate_tank_control_valve(tank, 1);
    }
    else if (*(tank->control_pres) > (tank->PID_ctrl_vlv_high_pres)) {
    	actuate_tank_control_valve(tank, 0);
    }
}


// TODO: refactor this bigly
void tank_startup_init_motor_position(TPC_Info* tank) {
	// These variables are all doubles because they need very high precision
	static double gamma  = 1.66;
	static double sg     = 0.137;
	static double c1     = 2834;
	static double c2     = 6140;
	static double c3     = 5360;
	static double c4     = 769.8;

	double crit_pr, t_r, valve_cv, t_f, /*p_rat, t_rat,*/ q_acf, q_scf, vdot;
	double deg = 0;

	double p_i    = (double)(*(tank->COPV_pres));       // cng pressure
	double p_o    = (double)(*(tank->control_pres));     // tank pressure

	// Avoid divide by zero error
	if (p_i == 0) {
		p_i = 0.1;
	}
	if (p_o == 0) {
		p_o = 0.1;
	}

	t_f = 300; // K  TODO: what are these, and why aren't they also static?
	double t_std = 288; // K
	double p_std = 14.7; // psi

	if (tank->is_cryogenic) {
		vdot   = 0.00317;
	}
	else {
		vdot   = 0.00361;
	}

	// Calculations
	crit_pr = pow(2.0 / (gamma + 1), gamma / (gamma - 1));
	t_r     = (double)(*(tank->COPV_temp)) * (9.0/5);
	q_acf = vdot*2118.88; // cfm

	if (tank->is_cryogenic) { // cryogenic liquid case
		q_scf = q_acf*p_o/(p_std)*1.3;
	}
	else {
		q_scf = q_acf*p_o*t_std/(p_std*t_f);
	}

	if (crit_pr >= p_o/p_i) { // Critical flows
		valve_cv = q_scf/13.61/p_i/sqrt(1.0/sg/t_r);
	} else {
		valve_cv = q_scf/16.05/sqrt((pow(p_i,2)-pow(p_o,2))/sg/t_r);
	}

	deg = init_motor_pos_deg_correction_factor * (c1*pow(valve_cv, 4) +
		  c2*pow(valve_cv, 3) +
		  c3*pow(valve_cv, 2) +
		  c4*valve_cv);

	// Output limiting
	if (deg < 0) { deg = 0; }
	if (deg > 2460) { deg = 2460; }
	// TODO: where does 2460 come from?

	// TODO: Why is the direction manually set here?
	// can it just be the shortest path?
	if (tank->tank_enable) {
		manual_stepper_pos_override[tank->motor_num] = 1;
		targetPos[tank->motor_num] = deg; // position given in deg
		curDir[tank->motor_num] = (curPos[tank->motor_num] < targetPos[tank->motor_num]) ? 1 : -1; // CCW facing the motor
		mtr_set[tank->motor_num] = deg; // save new motor position setpoint
	}
}

