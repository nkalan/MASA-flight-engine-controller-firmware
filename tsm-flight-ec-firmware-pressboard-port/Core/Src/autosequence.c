/*
 * autosequence.c
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#include "autosequence.h"
#include "globals.h"  // STATE enum and automatic abort toggle
//#include "valvelib.h"
#include "constants.h"  // sensor/actuator mappings
#include "status_flags.h"

//#define STEPPER_MOTOR_STEP_ANGLE   (1.8)
#define VALVE_ON                   (GPIO_PIN_SET)
#define VALVE_OFF                  (GPIO_PIN_RESET)

// Telem on/off toggle
extern uint8_t telem_disabled;

// Tanks
TPC_Info tanks[NUM_TANKS];

// Timings struct declared here
Autosequence_Info autosequence;

/**
 * This function must be called AFTER variables are read from flash
 */
void init_autosequence_constants() {
	// Hardcoded timings
	autosequence.startup_motor_start_delay_ms = 500;
	autosequence.hotfire_purge_off_time_ms = 50;
	autosequence.post_vent_on_time_ms = 1000;
	autosequence.post_vent_off_time_ms = 6000;
	autosequence.post_purge_off_time_ms = 10000;

	// Detection thresholds
	autosequence.ignition_ignitor_current_lower_bound = 0.3;  // TODO: fix this
	autosequence.ignition_ignitor_current_lower_bound_pass_min_detections = 30;  // 150ms

	// Nominal chamber pressure is 280psi
	autosequence.hotfire_chamber_pres_lower_bound = 70;                          // Nominal*0.25
	autosequence.hotfire_chamber_pres_lower_bound_pass_min_detections = 20;      // 100ms
	autosequence.hotfire_chamber_pres_lower_bound_abort_start_time_ms = 2000;    // Wait 2s into state before counting

	autosequence.hotfire_chamber_pres_upper_bound = 500;                         // Nominal * 1.8
	autosequence.hotfire_chamber_pres_upper_bound_pass_min_detections = 20;      // 100ms
}


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
	tanks[LOX_TANK_NUM].control_pres = &pressure[LOX_TANK_PRES_CH];  // TODO: change
	tanks[LOX_TANK_NUM].COPV_pres = &pressure[COPV_PRES_CH];  // TODO: change
	tanks[LOX_TANK_NUM].COPV_temp = &tc[COPV_TEMP_CH];
	tanks[LOX_TANK_NUM].PID_ctrl_loop_period_ms = 50;

	// Fuel tank configuration
	tanks[FUEL_TANK_NUM].is_cryogenic = 0;
	tanks[FUEL_TANK_NUM].control_valve_channel = FUEL_CONTROL_VALVE_CH;
	tanks[FUEL_TANK_NUM].control_pres = &pressure[FUEL_TANK_PRES_CH];  // TODO: change
	tanks[FUEL_TANK_NUM].COPV_pres = &pressure[COPV_PRES_CH];  // TODO: change
	tanks[FUEL_TANK_NUM].COPV_temp = &tc[COPV_TEMP_CH];
	tanks[FUEL_TANK_NUM].PID_ctrl_loop_period_ms = 50;

	// Motor info
	tanks[LOX_TANK_NUM].motor_num = LOX_TANK_NUM;
	tanks[LOX_TANK_NUM].motor_num = FUEL_TANK_NUM;

	// Bang bang thresholds
	tanks[LOX_TANK_NUM].bang_bang_low_pres_diff = 10;
	tanks[LOX_TANK_NUM].bang_bang_high_pres_diff = 15;
	tanks[FUEL_TANK_NUM].bang_bang_low_pres_diff = 5;
	tanks[FUEL_TANK_NUM].bang_bang_high_pres_diff = 10;
}

/**
 * Call this on initialization and every time you exit the autosequence
 */
void init_autosequence_control_variables() {
	autosequence.startup_init_motor_pos_complete = 0;
	autosequence.hotfire_lox_tank_enable_PID_control = 0;
	autosequence.hotfire_fuel_tank_enable_PID_control = 0;

	autosequence.ignition_ignitor_current_lower_bound_pass_count = 0;
	autosequence.hotfire_chamber_pres_lower_bound_pass_count = 0;
	autosequence.hotfire_chamber_pres_upper_bound_pass_count = 0;

	autosequence.ignition_ignitor_current_lower_bound_threshold_passed = 0;
	autosequence.hotfire_chamber_pres_lower_bound_threshold_passed = 0;
	autosequence.hotfire_chamber_pres_upper_bound_threshold_passed = 0;

	autosequence.post_gse_fuel_vent_signal = 0;
	autosequence.post_gse_fuel_vent_command_enable = 0;
}


/**
 * Call this function every time you want to abort.
 * Handles all actuations.
 */
void enter_abort_state() {
	// Enter Abort state
	// This line is for in case someone forgets to set Abort elsewhere
	STATE = Abort;

	// Close MPVs
	set_valve_channel(FUEL_MPV_PRESS_VALVE_CH, VALVE_OFF);
	set_valve_channel(FUEL_MPV_VENT_VALVE_CH, VALVE_OFF);
	set_valve_channel(LOX_MPV_VALVE_CH, VALVE_OFF);

	// Stop nozzle film cooling
	set_valve_channel(NOZZLE_FILM_COOLING_VALVE_CH, VALVE_OFF);

	// Close control valves
	set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_OFF);
	set_valve_channel(FUEL_CONTROL_VALVE_CH, VALVE_OFF);

	// De-energize ignitor
	set_valve_channel(IGNITOR_CH, VALVE_OFF);

	// Open vent valves
	// Fuel vent is handled by GSE controller
	set_valve_channel(LOX_TANK_VENT_VALVE_CH, VALVE_ON);
	autosequence.post_gse_fuel_vent_command_enable = 1;
	autosequence.post_gse_fuel_vent_signal = VALVE_ON;

	// Open purge valve
	set_valve_channel(PURGE_VALVE_CH, VALVE_ON);

	// Close motors (needle valves), 0 degrees should be closed.
	moveMotorToPos(0, LOX_TANK_NUM);
	moveMotorToPos(0, FUEL_TANK_NUM);

	// Stop TPC (not an actuation)
	autosequence.hotfire_lox_tank_enable_PID_control = 0;
	autosequence.hotfire_fuel_tank_enable_PID_control = 0;

	// Make sure telem is enabled, in case it aborted during
	// the MPV delay
	telem_disabled = 0;
}

/**
 * Call this function to safe the system.
 * Not the same as abort.
 *
 * Can only be called in AutoPress and Startup.
 */
void enter_safe_disarm_state() {
	// Including this line in case programmer forgets to set it elsewhere
	STATE = Safe;

	// Close control valves and go back to Manual.
	// TODO: should tank enable be looked at here?
	set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_OFF);
	set_valve_channel(FUEL_CONTROL_VALVE_CH, VALVE_OFF);

	// Reset all control variables whenever exiting the autosequence
	init_autosequence_control_variables();

	// TODO: go back to Manual?
}

/**
 * Keep a counter of consecutive times that the ematch current
 * is below a certain threshold while it's set high. If it
 * reaches a certain count, set a flag. Current should drop
 * out after the ematch burns and the ignitor lights.
 *
 *  This function is called every 5ms during the detection period.
 */
void update_ignitor_break_detector() {			  // Counter
	  if (ivlv[IGNITOR_CH] < autosequence.ignition_ignitor_current_lower_bound) {
		  ++autosequence.ignition_ignitor_current_lower_bound_pass_count;
	  }
	  else {
		  autosequence.ignition_ignitor_current_lower_bound_pass_count = 0;
	  }

	  // Threshold check
	  if (autosequence.ignition_ignitor_current_lower_bound_pass_count
			  >= autosequence.ignition_ignitor_current_lower_bound_pass_min_detections) {
		  autosequence.ignition_ignitor_current_lower_bound_threshold_passed = 1;
	  }
	  else {
		  autosequence.ignition_ignitor_current_lower_bound_threshold_passed = 0;
	  }
}


/**
 * Keep a counter of consecutive times the chamber pressure
 * is below a certain threshold (wait until after the startup
 * transient). If it reaches a certain count, set a flag.
 * Pressure should stay above this threshold during
 * a nominal hotfire.
 *
 * This function is called every 5ms during the detection period.
 */
void update_combustion_failure_detector() {
	  // Counter
	  if (pressure[CHAMBER_PRES_CH] < autosequence.hotfire_chamber_pres_lower_bound) {
		  ++autosequence.hotfire_chamber_pres_lower_bound_pass_count;
	  }
	  else {
		  autosequence.hotfire_chamber_pres_lower_bound_pass_count = 0;
	  }

	  // Threshold check
	  if (autosequence.hotfire_chamber_pres_lower_bound_pass_count
			  >= autosequence.hotfire_chamber_pres_lower_bound_pass_min_detections) {
		  autosequence.hotfire_chamber_pres_lower_bound_threshold_passed = 1;
	  }
	  else {
		  autosequence.hotfire_chamber_pres_lower_bound_threshold_passed = 0;
	  }
}


/**
 * Keep a counter of consecutive times the chamber pressure
 * is above a certain threshold. If it reaches a certain count,
 * set a flag. Pressure should stay below this threshold for
 * the entire hotfire.
 *
 * This function is called every 5ms during the detection period.
 */
void update_hard_start_detector() {
	// Counter
	if (pressure[CHAMBER_PRES_CH] > autosequence.hotfire_chamber_pres_upper_bound) {
		++autosequence.hotfire_chamber_pres_upper_bound_pass_count;
	}
	else {
		autosequence.hotfire_chamber_pres_upper_bound_pass_count = 0;
	}

	// Threshold check
	if (autosequence.hotfire_chamber_pres_upper_bound_pass_count
			>= autosequence.hotfire_chamber_pres_upper_bound_pass_min_detections) {
		autosequence.hotfire_chamber_pres_upper_bound_threshold_passed = 1;
	}
	else {
		autosequence.hotfire_chamber_pres_upper_bound_threshold_passed = 0;
	}
}


void manual_state_transition(uint8_t next_state) {

	// Aborts work in any state
	if (next_state == Abort) {
		STATE = Abort;
		enter_abort_state();
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
		}
	}
	else if (STATE == AutoPress) {
		if (next_state == Manual) {
			STATE = Safe;
			enter_safe_disarm_state();
		}
		else if (next_state == Startup) {
			// Close control valves
			set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_OFF);
			set_valve_channel(FUEL_CONTROL_VALVE_CH, VALVE_OFF);

			autosequence.startup_start_time_ms = SYS_MILLIS;
			STATE = Startup;
		}
	}
	else if (STATE == Startup) {
		if (next_state == Manual) {
			STATE = Safe;
			enter_safe_disarm_state();
		}
		// GUI sends continue command for some reason
		else if (next_state == Continue) {
			// Only allow ignition after initial motor position is handled
			if (autosequence.startup_init_motor_pos_complete) {
				autosequence.startup_init_motor_pos_complete = 0;  // Reset flag
				STATE = Ignition;
				autosequence.ignition_start_time_ms = SYS_MILLIS;
				set_valve_channel(PURGE_VALVE_CH, VALVE_ON);  // Turn purge on
			}
		}
	}
	else if (STATE == IgnitionFail) {
		if (next_state == Manual) {
			init_autosequence_control_variables();
			STATE = Manual;  // Operator must dismiss IgnitionFail condition
		}
	}
	else if (STATE == Abort) {
		if (next_state == Manual) {
			init_autosequence_control_variables();
			// Stop venting fuel tank
			autosequence.post_gse_fuel_vent_command_enable = 0;
			STATE = Manual;  // Operator must dismiss Abort condition
		}
	}
}

/**
 * Only works for Startup, Ignition, Hotfire, and Post
 */
uint32_t get_ellapsed_time_in_autosequence_state_ms() {
	if (STATE == Startup) {
		return SYS_MILLIS - autosequence.startup_start_time_ms;
	}
	else if (STATE == Ignition) {
		return SYS_MILLIS - autosequence.ignition_start_time_ms;
	}
	else if (STATE == Hotfire) {
		return SYS_MILLIS - autosequence.hotfire_start_time_ms;
	}
	else if (STATE == Post) {
		return SYS_MILLIS - autosequence.post_start_time_ms;
	}
	else {
		return 0;
	}
}

/*
 * Only Ignition, Hotfire, and Post have defined time limits
 */
uint32_t get_remaining_time_in_autosequence_state(uint32_t T_state) {
	if (STATE == Ignition) {
		return (autosequence.ignition_ignitor_on_delay_ms
				+ autosequence.ignition_ignitor_high_duration_ms) - T_state;
	}
	else if (STATE == Hotfire) {
		return autosequence.hotfire_test_duration_ms - T_state;
	}
	else if (STATE == Post) {
		return autosequence.post_purge_off_time_ms - T_state;
	}
	else {
		return 0;
	}
}

/**
 * Called every main while loop
 */
void execute_autosequence() {

	// Autosequence timings are done relative to the start of the state
	autosequence.T_state = get_ellapsed_time_in_autosequence_state_ms();

	// Update time remaining in state for GUI
	state_rem_duration =
			get_remaining_time_in_autosequence_state(autosequence.T_state);

	// Doesn't use if else within each state in case the timings overlap
	if (STATE == Ignition) {
		// Purge should've turned on when entering Ignition
		// Wait for the delay, then turn ignitor on
		if (autosequence.T_state >= autosequence.ignition_ignitor_on_delay_ms) {
			set_valve_channel(IGNITOR_CH, VALVE_ON);
		}
		// Hold ignitor high for a certain amount of time
		if (autosequence.T_state >= autosequence.ignition_ignitor_on_delay_ms
				+ autosequence.ignition_ignitor_high_duration_ms) {

			// Only proceed to Hotfire if the ignitor break is detected
			// and automatic aborts are enabled
			if (autosequence.enable_auto_aborts &&
					!autosequence.ignition_ignitor_current_lower_bound_threshold_passed) {
				// Transition to IgnitionFail state
				STATE = IgnitionFail;

				// Reset all control variables whenever exiting the autosequence
				autosequence.ignition_ignitor_current_lower_bound_threshold_passed = 0;  // Not needed
				init_autosequence_control_variables();

				// Reset all control variables whenever exiting the autosequence
				init_autosequence_control_variables();

				// Close control valves
				set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_OFF);
				set_valve_channel(FUEL_CONTROL_VALVE_CH, VALVE_OFF);

				// Ignitor low
				set_valve_channel(IGNITOR_CH, VALVE_OFF);

				// Purge low
				set_valve_channel(PURGE_VALVE_CH, VALVE_OFF);

				// Now wait for operator to go to Manual
			}
			// Successful ignitor break OR auto aborts disabled
			else {
				// Transition to Hotfire state
				STATE = Hotfire;
				autosequence.hotfire_start_time_ms = SYS_MILLIS;

				// Ignitor low
				set_valve_channel(IGNITOR_CH, VALVE_OFF);

				// Open LOX MPV
				set_valve_channel(LOX_MPV_VALVE_CH, VALVE_ON);

				// Open LOX control valve
				set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_ON);

				// Disable telemetry to prevent telem from blocking valve
				// timings to prevent a hard start
				// TODO: get rid of this when DMA tx is working?
				telem_disabled = 1;
			}
		}
	}

	else if (STATE == Hotfire) {
		// Tank pressure control periodic function calls handled in main()
		// Not using else if in case the timings overlap

		// Automatic abort cases
		if (autosequence.enable_auto_aborts) {
			// Chamber pressure too low - only active 1s after Hotfire
			// see update_combustion_failure() and its call in main()
			// Double check that it waits until after the startup transient
			if (autosequence.T_state > autosequence.hotfire_chamber_pres_lower_bound_abort_start_time_ms
					&& autosequence.hotfire_chamber_pres_lower_bound_threshold_passed) {
				// Reset abort flag
				autosequence.hotfire_chamber_pres_lower_bound_threshold_passed = 0;

				// Handle abort
				STATE = Abort;
				enter_abort_state();
				set_status_flag(EC_FLAG_ABORT_CHMBR_PRES_LOW);
				return;  // Stop other valves from actuating this loop
			}

			// Chamber pressure too high - active through entire Hotfire
			// Aborts on first instance of detection to catch hard starts
			if (pressure[CHAMBER_PRES_CH]
						 > autosequence.hotfire_chamber_pres_upper_bound) {
				STATE = Abort;
				enter_abort_state();
				set_status_flag(EC_FLAG_ABORT_CHMBR_PRES_HIGH);
				return;  // Stop other valves from actuating this loop
			}
		}

		// Turn on LOX pressure control
		// Relative to 0 because LOX leads
		if (autosequence.T_state >= (0 + autosequence.hotfire_pid_start_delay_ms)
				&& !autosequence.hotfire_lox_tank_enable_PID_control) {
			// Should only get called once when it starts pressure control
			tank_init_control_loop(&tanks[LOX_TANK_NUM]);
			autosequence.hotfire_lox_tank_enable_PID_control = 1;
		}

		// Fuel on
		if (autosequence.T_state >= autosequence.hotfire_fuel_mpv_delay_ms) {
			// Open Fuel MPV (Press AND Vent)
			set_valve_channel(FUEL_MPV_VENT_VALVE_CH, VALVE_ON);
			set_valve_channel(FUEL_MPV_PRESS_VALVE_CH, VALVE_ON);

			// Open Fuel control valve
			set_valve_channel(FUEL_CONTROL_VALVE_CH, VALVE_ON);

			// Re-enable telemetry
			// TODO: remove this when DMA tx is working?
			telem_disabled = 0;
		}

		// Turn on Fuel pressure control
		// Delay is relative to MPV opening
		if (autosequence.T_state >= (autosequence.hotfire_fuel_mpv_delay_ms
				+ autosequence.hotfire_pid_start_delay_ms)
				&& !autosequence.hotfire_fuel_tank_enable_PID_control) {
			// Should only get called once when it starts pressure control
			tank_init_control_loop(&tanks[FUEL_TANK_NUM]);
			autosequence.hotfire_fuel_tank_enable_PID_control = 1;
		}

		// Nozzle film cooling timing is relative to LOX MPV opening
		if (autosequence.T_state >= autosequence.hotfire_film_cooling_on_time_ms) {
			// Nozzle film cooling on
			set_valve_channel(NOZZLE_FILM_COOLING_VALVE_CH, VALVE_ON);
		}

		// After combustion starts, turn off purge
		if (autosequence.T_state >= autosequence.hotfire_purge_off_time_ms) {
			// Purge low
			set_valve_channel(PURGE_VALVE_CH, VALVE_OFF);
		}

		// Stop hotfire
		if (autosequence.T_state >= autosequence.hotfire_test_duration_ms) {
			// Transition to Post state
			STATE = Post;
			autosequence.post_start_time_ms = SYS_MILLIS;

			// Close LOX and Fuel MPVs
			set_valve_channel(FUEL_MPV_PRESS_VALVE_CH, VALVE_OFF);
			set_valve_channel(FUEL_MPV_VENT_VALVE_CH, VALVE_OFF);
			set_valve_channel(LOX_MPV_VALVE_CH, VALVE_OFF);

			// Nozzle film cooling off
			set_valve_channel(NOZZLE_FILM_COOLING_VALVE_CH, VALVE_OFF);

			// Close control valves
			set_valve_channel(LOX_CONTROL_VALVE_CH, VALVE_OFF);
			set_valve_channel(FUEL_CONTROL_VALVE_CH, VALVE_OFF);

			// Stop tank pressure control
			autosequence.hotfire_lox_tank_enable_PID_control = 0;
			autosequence.hotfire_fuel_tank_enable_PID_control = 0;

			// Purge high
			set_valve_channel(PURGE_VALVE_CH, VALVE_ON);
		}
	}

	else if (STATE == Post) {
		// MPVs should already be closed and purge should've started by now

		if (autosequence.T_state >= autosequence.post_vent_on_time_ms) {
			// Vent both tanks
			set_valve_channel(LOX_TANK_VENT_VALVE_CH, VALVE_ON);

			// Fuel vent handled by GSE controller
			autosequence.post_gse_fuel_vent_command_enable = 1;
			autosequence.post_gse_fuel_vent_signal = VALVE_ON;
		}
		if (autosequence.T_state >= autosequence.post_vent_off_time_ms) {
			// Close tank vents
			set_valve_channel(LOX_TANK_VENT_VALVE_CH, VALVE_OFF);

			// Fuel vent handled by GSE controller
			autosequence.post_gse_fuel_vent_command_enable = 1;
			autosequence.post_gse_fuel_vent_signal = VALVE_OFF;
		}
		if (autosequence.T_state >= autosequence.post_purge_off_time_ms) {
			// Purge low
			set_valve_channel(PURGE_VALVE_CH, VALVE_OFF);

			// Stop valve commands to the GSE controller
			autosequence.post_gse_fuel_vent_command_enable = 0;
			autosequence.post_gse_fuel_vent_signal = VALVE_OFF;

			// Reset all control variables whenever exiting the autosequence
			init_autosequence_control_variables();

			// Transition back to Manual
			STATE = Manual;
		}
	}

	else if (STATE == Safe) {
		// Safing the system should've already happened
		// lmao what do you even do here?
		STATE = Manual;
	}

	else if (STATE == IgnitionFail) {
		// Wait for the operator to go back to Manual.
	}

	else if (STATE == Abort) {
		// Actuations should've been handled already during state transition.
		// Wait for the operator to go back to Manual.

		// To catch any additional bugs in the state transition,
		// Constantly actuate the proper valves during the Abort state.
		enter_abort_state();
	}

	// To catch bugs related to the Manual transition not
	// shutting down the GSE controller fuel vent valve,
	// constantly refresh those control variables
	if (STATE != Post && STATE != Abort) {
		autosequence.post_gse_fuel_vent_command_enable = 0;
		autosequence.post_gse_fuel_vent_signal = 0;
	}
}
