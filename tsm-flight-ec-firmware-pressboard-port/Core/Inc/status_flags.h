/*
 * status_flags.h
 *
 *  Created on: Jun 28, 2021
 *      Author: natha
 */

#ifndef INC_STATUS_FLAGS_H_
#define INC_STATUS_FLAGS_H_

#include <stdint.h>

// Limit of 32 (0 to 31 incl.) different flags
#define EC_FLAG_COMMAND_DROP                (0)
#define EC_FLAG_ABORT_MANUAL                (1)
#define EC_FLAG_ABORT_CHMBR_PRES_LOW        (2)
#define EC_FLAG_ABORT_CHMBR_PRES_HIGH       (3)
#define EC_FLAG_FAILED_IGNTN                (4)
#define EC_FLAG_QD_INTACT                   (5)  // atlo or av qd?
#define EC_FLAG_FUEL_PT_A_FAIL              (6)
#define EC_FLAG_FUEL_PT_B_FAIL              (7)
#define EC_FLAG_FUEL_PT_C_FAIL              (8)
#define EC_FLAG_FUEL_PT_MULTIPLE_FAIL       (9)
#define EC_FLAG_LOX_PT_A_FAIL              (10)
#define EC_FLAG_LOX_PT_B_FAIL              (11)
#define EC_FLAG_LOX_PT_C_FAIL              (12)
#define EC_FLAG_LOX_PT_MULTIPLE_FAIL       (13)
#define EC_FLAG_COPV_PT_A_FAIL             (14)
#define EC_FLAG_COPV_PT_B_FAIL             (15)
#define EC_FLAG_COPV_PT_C_FAIL             (16)
#define EC_FLAG_COPV_PT_MULTIPLE_FAIL      (17)

extern uint32_t status_flags;  // globals.h

/**
 * Sets the specified status bit high in the status_flags variable.
 * @param One of the EC_FLAG_ constants to represent a status bit.
 */
void set_status_flag(uint32_t flag);

/**
 * Sets the specified status bit low in the status_flags variable.
 * @param One of the EC_FLAG_ constants to represent a status bit.
 */
void remove_status_flag(uint32_t flag);

#endif /* INC_STATUS_FLAGS_H_ */
