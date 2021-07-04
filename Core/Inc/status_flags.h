/*
 * status_flags.h
 *
 *  Created on: Jun 28, 2021
 *      Author: natha
 */

#ifndef INC_STATUS_FLAGS_H_
#define INC_STATUS_FLAGS_H_

// Limit of 32 (0 to 31 incl.) different flags
#define EC_FLAG_COMMAND_DROP                (0)
#define EC_FLAG_ABORT_MANUAL                (1)
#define EC_FLAG_ABORT_CHMBR_PRES_LOW        (2)
#define EC_FLAG_ABORT_CHMBR_PRES_HIGH       (3)
#define EC_FLAG_FAILED_IGNTN                (4)
#define EC_FLAG_QD_INTACT                   (5)  // atlo or av qd?

extern uint32_t status_flags;  // globals.h

// Implemented in .h since they're so short; no .c file
void remove_status_flags() {
	status_flags = 0;
}

void set_status_flag(uint32_t flag) {
	status_flags |= (1 << flag);  // Set that bit index to 1
}

void remove_status_flag(uint32_t flag) {
	status_flags &= ~(1 << flag);  // Remove that bit index
}

#endif /* INC_STATUS_FLAGS_H_ */
