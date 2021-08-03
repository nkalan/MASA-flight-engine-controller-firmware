/*
 * tank_pressure_control.c
 *
 *  Created on: Jun 22, 2021
 *      Author: natha
 */


#include "tank_pressure_control.h"
#include "constants.h"
#include "valves.h"
#include "math.h"

extern float init_motor_pos_deg_correction_factor;  // from globals


/**
 * Small wrapper around motor actuation, to allow specific
 * tanks to be disabled. Calculations are always run, but actuations
 * are stopped when tank_enable is false.
 */
void actuate_tank_motor_pos(TPC_Info* tank, float motor_pos) {
	if (tank->tank_enable) {
		// actuate motor to pos
	}
}

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


// copied from press board
/*
void doCalculations(uint8_t motor_num) {
    static float next_errSum        = 0;
    static float pid_change         = 0;
    static float maxDiff            = 0;

    if (do_calculations[motor_num]) { //for timer
        //-----------------------//
        //Control Loop

        // PID error term calculations
        maxDiff = maxPos - curPos[motor_num]; //distance from maxPos
        timeChange = control_loop_period; //in seconds
        error = tank_tar_pres[motor_num] - *controlPres[motor_num];

        next_errSum = errSum[motor_num] + (error * timeChange);
        kp_term =  mtr_kp[motor_num] * error;
        ki_term = mtr_ki[motor_num] * next_errSum;
        kd_term = mtr_kd[motor_num] * (error - last_error[motor_num]) / timeChange;
        pid_change = kp_term + ki_term + kd_term; //total change

        // Next state update
        if (pid_change < -curPos[motor_num]) { //min cap
            delta = -curPos[motor_num];
        } else if (pid_change > maxDiff) { //max cap
            delta = maxDiff;
        } else {
            delta = pid_change;
            errSum[motor_num] = next_errSum; // perform errSum update iff not saturated
        }
        targetPos[motor_num] = curPos[motor_num] + delta;

        // Stepping Actuation
        curDir[motor_num] = (curPos[motor_num] < targetPos[motor_num]) ? 1 : -1;

        last_error[motor_num] = error;

        // Saving variables to telem
        mtr_set[motor_num] = targetPos[motor_num];
        mtr_kp_err[motor_num] = kp_term;
        mtr_ki_err[motor_num] = ki_term;
        mtr_kd_err[motor_num] = kd_term;

        do_calculations[motor_num] = 0; //reset timer
    }
}
*/

/*
void init_control_variables() {
    for (uint8_t i = 0 ; i < NUM_TANKS; ++i) {
        last_error[i] = Setpoint[i];
        curDir[i] = 0; // change to one to enable motor spinning manually
        if (!is_flash_constant_good) {
            mtr_kp[i] = 9.75;
            mtr_ki[i] = 3.75;
            mtr_kd[i] = 2;        // dont need to be reset before running each loop

            tank_tar_pres[i] = Setpoint[i]; // Redundant but still keep in case flash doesn't work
            tank_low_pres[i] = Setpoint[i]*0.95f;
            tank_high_pres[i]= Setpoint[i]*1.1f;

            pass_low_tol[i] = tank_low_pres[i];
            pass_high_tol[i] = tank_high_pres[i];
        }
    }

    // init stepper ramping variables

    //stepper.SPS = 1000; // start motor 1000 steps per second
    //stepper.SPS_target = 2000; // target steps/sec
    //stepper.tim_osc = 1000000; // tim6 update speed
    //stepper.SPS_tim_reg = (uint16_t)(stepper.tim_osc / stepper.SPS);
    //stepper.acc_res = 2; // increments SPS by step res
    //stepper.acc_step = 0;

    // ^ These should be configured by the motor library

    //control_loop_period = ( 1.0/1000 ) * (__HAL_TIM_GET_AUTORELOAD(&htim10)+1);

    //test_duration = 4000; // test duration four seconds
}
*/

/*
// Note this function should be called automatically right before the test
void reset_control_variables(int32_t test_start_time, uint8_t tank_num) {
    // PID variables reset

    errSum[tank_num] = 0;     // I term
    last_error[tank_num] = tank_tar_pres[tank_num] - *controlPres[tank_num]; // D term
    kp_term = 0;
    ki_term = 0;
    kd_term = 0;

    curDir[tank_num] = 0; // set initial motor direction

    // Reset targetPos from curPos
    targetPos[tank_num] = curPos[tank_num];

    // Update values for telem
    mtr_kp_err[tank_num] = kp_term;
    mtr_ki_err[tank_num] = ki_term;
    mtr_kd_err[tank_num] = kd_term;
}
*/


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
	// TODO: define these 2 motor variables somewhere, probably motor struct
	// TODO: change all these variable names when they get decided
	float motor_pos, max_motor_pos;
	float max_motor_delta = max_motor_pos - motor_pos;
	if (PID_total_output < -motor_pos) {  // Lower bound
		motor_delta = -motor_pos;
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
	actuate_tank_motor_pos(tank, motor_pos + motor_delta);

	// Log data
	// TODO: should this go here, or in a different function?
	// Maybe just update some struct variables and log them later
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

	double crit_pr, t_r, valve_cv, t_f, p_rat, t_rat, q_acf, q_scf, vdot;
	double deg = 0;

	double p_i    = (double)(*(tank->COPV_pres));       // cng pressure
	double p_o    = (double)(*(tank->control_pres));     // tank pressure

	// Avoid divide by zero error
	if (p_i == 0) {
		p_i = 0.0000001;
	}
	if (p_o == 0) {
		p_o = 0.0000001;
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
	/*
	if (is_tank_enabled[tank_num]) {
		manual_stepper_pos_override[tank_num] = 1;
		targetPos[tank_num] = deg; // position given in deg
		curDir[tank_num] = (curPos[tank_num] < targetPos[tank_num]) ? 1 : -1; // CCW facing the motor
		mtr_set[tank_num] = deg; // save new motor position setpoint
	}
	*/
}

