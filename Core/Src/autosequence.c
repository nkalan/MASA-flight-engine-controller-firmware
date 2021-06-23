/*
 * autosequence.c
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */

#include "autosequence.h"
#include "globals.h"  // STATE struct definition
//extern enum States STATE;

void manual_state_transition(uint8_t next_state) {

	// Aborts work in any state
	if (next_state == Abort) {
		STATE = Abort;
		// TODO: handle abort case here?
		return;
	}

	if (STATE == Manual) {
		if (next_state == Armed) {
			STATE == Armed;
		}
	}
	else if (STATE == Armed) {
		if (next_state == Manual) {
			STATE == Manual;
		}
		else if (next_state == AutoPress) {
			STATE == AutoPress;
			// TODO: handle Autopress transition here?
		}
	}
	else if (STATE == AutoPress) {
		if (next_state == Manual) {
			STATE == Safe;
		}
		// TODO: manual override into Startup
	}
	else if (STATE == Startup) {
		if (next_state == Manual) {
			STATE == Safe;
		}
		// TODO: manual transition into Ignition
	}
	else if (STATE == IgnitionFail) {
		if (next_state == Manual) {
			STATE == Manual;
			// TODO: how do you prevent it from going to Manual before
			// the shutdown sequence finishes?
		}
	}
	else if (STATE == Abort) {
		if (next_state == Manual) {
			STATE == Manual;  // Operator must dismiss Abort condition
		}
	}
}
