/*
 * autosequence.c
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#include "autosequence.h"
#include "globals.h"  // STATE enum


// declare Autosequence struct
Autosequence autosequence;

void init_autosequence_timings() {

}

void manual_state_transition(uint8_t next_state) {

	// Aborts work in any state
	if (next_state == Abort) {
		STATE = Abort;
		// TODO: handle abort case here?
		return;
	}

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
		// TODO: manual transition into Ignition (not Continue please)
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

void execute_autosequence() {

	if (STATE == AutoPress) {

	}
	else if (STATE == Ignition) {

	}
	else if (STATE == IgnitionFail) {





		// Prevents the operator from going back to Manual, if they somehow
		// give the Manual command before this finishes executing
		// if (done with whatever) {
			autosequence.ignition_failure_shutdown_flag = 1;
		// }
	}
	else if (STATE == Hotfire) {

	}
	else if (STATE == Post) {

	}
	else if (STATE == Safe) {

	}
	else if (STATE == Abort) {

	}
}
