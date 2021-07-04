/*
 * tank_pressure_control.c
 *
 *  Created on: Jun 22, 2021
 *      Author: natha
 */


#include "tank_pressure_control.h"



// Psi toggle point ranges
// TODO: move these somewhere else?

void set_tank_motor_pos(TPC_Info* tank, float motor_pos) {
	if (tank->tank_enable) {
		// actuate motor to pos
	}
}

void set_tank_control_valve(TPC_Info* tank /*, control valve state */) {
	if (tank->tank_enable) {
		// set control valve to state
	}
}

// copied from press board
void checkControlValves(uint8_t tank_num) {
	/*
    if (do_control_valve_check[tank_num]) {
        // Execute control solenoid safety loop
        if (*controlPres[tank_num] < tank_low_pres[tank_num]){
            setValve(vlv_assignments.ControlValves[tank_num], OpenVlv);
        } else if (*controlPres[tank_num] > tank_high_pres[tank_num]){
            setValve(vlv_assignments.ControlValves[tank_num], CloseVlv);
        }
        do_control_valve_check[tank_num] = 0;
    }
    */
}

// copied from press board
void doCalculations(uint8_t motor_num) {
    static float next_errSum        = 0;
    static float pid_change         = 0;
    static float maxDiff            = 0;

    /*
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
	*/
}

void tank_autopress_bang_bang(TPC_Info* tank) {
	if ( ) {

	}
}
