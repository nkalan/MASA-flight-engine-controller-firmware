/*
 * board_commands.h
 *
 *  Created on: Aug 6, 2021
 *      Author: natha
 */


#ifndef INC_BOARD_COMMANDS_H_
#define INC_BOARD_COMMANDS_H_

#include <stdint.h>

void send_gse_set_vlv_cmd(uint32_t vlv_num, uint8_t vlv_state);


#endif /* INC_BOARD_COMMANDS_H_ */
