/*
 * autosequence.c
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#include "autosequence.h"
#include "globals.h"  // STATE enum
#include "valvelib.h"
#include "constants.h"  // sensor/actuator mappings
#include "status_flags.h"

#define STEPPER_MOTOR_STEP_ANGLE   (1.8)
#define VALVE_ON                   (GPIO_PIN_SET)
#define VALVE_OFF                  (GPIO_PIN_RESET)


// Telem on/off toggle
extern uint8_t telem_disabled;

// Tanks
TPC_Info tanks[NUM_TANKS];


/**
 * Configuring tank settings that don't change between tests.
 * Variables stored in nonvolatile memory are initialized elsewhere.
 *
 * TODO: say where they're initialized.
 */
void init_tank_pressure_control_configuration() {

	// LOX tank configuration
	tanks[LOX_TANK_NUM].is_cryogenic = 1;
	tanks[LOX_TANK_NUM].control_valve_channel = LOX_CONTROL_VALVE_CH;
	tanks[LOX_TANK_NUM].control_pres = &lox_control_pressure;
	tanks[LOX_TANK_NUM].COPV_pres = &copv_control_pressure;
	tanks[LOX_TANK_NUM].COPV_temp = &tc[COPV_TEMP_CH];
	tanks[LOX_TANK_NUM].PID_ctrl_loop_period_ms = 50;

	// Fuel tank configuration
	tanks[FUEL_TANK_NUM].is_cryogenic = 0;
	tanks[FUEL_TANK_NUM].control_valve_channel = FUEL_CONTROL_VALVE_CH;
	tanks[FUEL_TANK_NUM].control_pres = &fuel_control_pressure;
	tanks[FUEL_TANK_NUM].COPV_pres = &copv_control_pressure;
	tanks[FUEL_TANK_NUM].COPV_temp = &tc[COPV_TEMP_CH];
	tanks[FUEL_TANK_NUM].PID_ctrl_loop_period_ms = 50;

	// Motor configuration
	L6470_init_motor(&(tanks[LOX_TANK_NUM].motor), L6470_FULL_STEP_MODE,
			STEPPER_MOTOR_STEP_ANGLE);
	L6470_init_motor(&(tanks[FUEL_TANK_NUM].motor), L6470_FULL_STEP_MODE,
			STEPPER_MOTOR_STEP_ANGLE);

	// TODO: what else here?
}

/**
 * Timings struct declared here
 */
Autosequence_Timings autosequence;


/**
 * Call this function every time you want to abort.
 * Handles all actuations.
 */
void handle_abort_case_actuations() {
	// TODO: check normally open/normally closed on ALL these valves

	// Close MPVs
	set_valve_channel(MPV_PRESS_VALVE_CH, VALVE_OFF);
	set_valve_channel(LOX_MPV_VENT_VALVE_CH, VALVE_OFF);
	set_valve_channel(FUEL_MPV_VENT_VALVE_CH, VALVE_OFF);

	// Close control valves
	set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_OFF);
	set_valve_channel(FUEL_CONTROL_VALVE_CH, VALVE_OFF);

	// De-energize ignitor
	set_valve_channel(IGNITOR_CH, VALVE_OFF);

	// Open vent valves
	set_valve_channel(FUEL_TANK_VENT_VALVE_CH, VALVE_ON);
	set_valve_channel(LOX_TANK_VENT_VALVE_CH, VALVE_ON);

	// Open purge valve
	set_valve_channel(MPV_PURGE_VALVE_CH, VALVE_ON);

	// Close motors (needle valves), 0 degrees should be closed.
	L6470_goto_motor_pos(&(tanks[LOX_TANK_NUM].motor), 0);
	L6470_goto_motor_pos(&(tanks[FUEL_TANK_NUM].motor), 0);

	// Stop TPC (not an actuation)
	autosequence.hotfire_lox_tank_enable_PID_control = 0;
	autosequence.hotfire_fuel_tank_enable_PID_control = 0;
}

/**
 * Call this function to safe the system.
 * Not the same as abort.
 *
 * Can only be called in AutoPress and Startup.
 */
void handle_safe_disarm_actuations() {
	// Close control valves and go back to Manual.
	// TODO: should tank enable be looked at here?
	set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_OFF);
	set_valve_channel(FUEL_CONTROL_VALVE_CH, VALVE_OFF);

	// TODO: go back to Manual?
}

void manual_state_transition(uint8_t next_state) {

	// Aborts work in any state
	if (next_state == Abort) {
		STATE = Abort;
		handle_abort_case_actuations();
		set_status_flag(EC_FLAG_ABORT_MANUAL);
		return;
	}

	// Check current state to determine next state transition
	if (STATE == Manual) {
		if (next_state == Armed) {
			STATE = Armed;
		}
	}
	else if (STATE == Armed) {
		if (next_state == Manual) {
			STATE = Manual;
		}
		else if (next_state == AutoPress) {
			STATE = AutoPress;
			// TODO: handle Autopress transition here?
		}
	}
	else if (STATE == AutoPress) {
		if (next_state == Manual) {
			STATE = Safe;
		}
		// TODO: manual override into Startup (not sure about timing or command)
	}
	else if (STATE == Startup) {
		if (next_state == Manual) {
			STATE = Safe;
		}
		else if (next_state == Ignition) {
			STATE = Ignition;
			// Turn purge on
			set_valve_channel(MPV_PURGE_VALVE_CH, VALVE_ON);
		}
	}
	else if (STATE == IgnitionFail) {
		if (next_state == Manual && autosequence.ignition_failure_shutdown_flag) {
			STATE = Manual;
		}
	}
	else if (STATE == Abort) {
		if (next_state == Manual) {
			STATE = Manual;  // Operator must dismiss Abort condition
		}
	}
}

void execute_autosequence(int32_t T_state, Autosequence_Timings* autosequence) {
	// Doesn't use if else in case the timings overlap

	if (STATE == Ignition) {
		// Purge should've turned on when entering Ignition
		if (T_state >= autosequence->ignition_ignitor_on_time_ms) {
			set_valve_channel(IGNITOR_CH, VALVE_ON);
		}
		if (T_state >= autosequence->ignition_ignitor_off_time_ms) {
			// Ignitor low
			set_valve_channel(IGNITOR_CH, VALVE_OFF);

			// Open LOX MPV
			set_valve_channel(MPV_PRESS_VALVE_CH, VALVE_ON);
			set_valve_channel(LOX_MPV_VENT_VALVE_CH, VALVE_ON);

			// Open LOX control valve
			set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_ON);

			// Disable telemetry to prevent telem from blocking valve timings
			// to prevent a hard start
			// TODO: get rid of this when DMA tx is working?
			telem_disabled = 1;

			// Transition to Hotfire state
			STATE = Hotfire;
		}
	}

	else if (STATE == IgnitionFail) {
		// Prevents the operator from going back to Manual, if they somehow
		// give the Manual command before this finishes executing
		// if (done with whatever) {
			autosequence->ignition_failure_shutdown_flag = 1;
		// }
	}

	else if (STATE == Hotfire) {
		// Tank pressure control periodic function calls handled in main()
		// Not using else if in case the timings overlap

		if (T_state >= autosequence->hotfire_lox_PID_control_start_time_ms
				&& autosequence->hotfire_lox_tank_enable_PID_control) {
			// Should only get called once when it starts pressure control
			tank_init_control_loop(&tanks[LOX_TANK_NUM]);
			autosequence->hotfire_lox_tank_enable_PID_control = 1;
		}
		if (T_state >= autosequence->hotfire_fuel_on_time_ms) {
			// Open Fuel MPV (MPV Press should already be open)
			set_valve_channel(FUEL_MPV_VENT_VALVE_CH, VALVE_ON);

			// Re-enable telemetry
			// TODO: remove this when DMA tx is working?
			telem_disabled = 0;
		}
		if (T_state >= autosequence->hotfire_fuel_PID_control_start_time_ms
			&& !autosequence->hotfire_fuel_tank_enable_PID_control) {
			// Should only get called once when it starts pressure control
			tank_init_control_loop(&tanks[FUEL_TANK_NUM]);
			autosequence->hotfire_fuel_tank_enable_PID_control = 1;
		}
		if (T_state >= autosequence->hotfire_purge_off_time_ms) {
			// Purge low
			set_valve_channel(MPV_PURGE_VALVE_CH, VALVE_OFF);
		}
		if (T_state >= autosequence->hotfire_film_cooling_on_time_ms) {
			// Nozzle film cooling on
			set_valve_channel(NOZZLE_FILM_COOLING_VALVE_CH, VALVE_ON);
		}
		if (T_state >= autosequence->hotfire_complete_time_ms) {
			// Close LOX and Fuel MPVs
			set_valve_channel(MPV_PRESS_VALVE_CH, VALVE_OFF);
			set_valve_channel(LOX_MPV_VENT_VALVE_CH, VALVE_OFF);
			set_valve_channel(FUEL_MPV_VENT_VALVE_CH, VALVE_OFF);

			// Purge high
			set_valve_channel(MPV_PURGE_VALVE_CH, VALVE_ON);

			// Nozzle film cooling off
			set_valve_channel(NOZZLE_FILM_COOLING_VALVE_CH, VALVE_OFF);

			// Stop tank pressure control
			tank_end_control_loop(&tanks[LOX_TANK_NUM]);
			tank_end_control_loop(&tanks[FUEL_TANK_NUM]);

			// Transition to Post state
			STATE = Post;
		}
	}

	else if (STATE == Post) {
		// MPVs should already be closed and purge should've started by now

		if (T_state >= autosequence->post_vent_on_time_ms) {
			// Vent both tanks
			set_valve_channel(LOX_TANK_VENT_VALVE_CH, VALVE_ON);
			set_valve_channel(FUEL_TANK_VENT_VALVE_CH, VALVE_ON);
		}
		if (T_state >= autosequence->post_vent_off_time_ms) {
			// Close tank vents
			set_valve_channel(LOX_TANK_VENT_VALVE_CH, VALVE_OFF);
			set_valve_channel(FUEL_TANK_VENT_VALVE_CH, VALVE_OFF);
		}
		if (T_state >= autosequence->post_purge_off_time_ms) {
			// Purge low
			set_valve_channel(MPV_PURGE_VALVE_CH, VALVE_OFF);

			// Transition back to Manual
			STATE = Manual;
		}
	}

	else if (STATE == Safe) {
		// Safing the system should've already happened
		// lmao what do you even do here?
		STATE = Manual;
	}

	else if (STATE == Abort) {
		// Actuations should've been handled already during state transition.
		// Wait for the operator to go back to Manual.
	}
}
