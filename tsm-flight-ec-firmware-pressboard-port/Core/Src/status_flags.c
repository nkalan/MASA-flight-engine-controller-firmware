/*
 * status_flags.c
 *
 *  Created on: Jul 5, 2021
 *      Author: natha
 */


#include "status_flags.h"

void set_status_flag(uint32_t flag) {
	status_flags |= (1 << flag);  // Set that bit index to 1
}

void remove_status_flag(uint32_t flag) {
	status_flags &= ~(1 << flag);  // Remove that bit index
}
