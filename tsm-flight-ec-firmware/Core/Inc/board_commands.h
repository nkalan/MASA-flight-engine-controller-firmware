/*
 * board_commands.h
 *
 *  Created on: Aug 6, 2021
 *      Author: natha
 */

#ifndef INC_BOARD_COMMANDS_H_
#define INC_BOARD_COMMANDS_H_

#include "constants.h"
#include "comms.h"

#define VLV_CMD_SZ (17)

/**
 * Pack and send a valve command to the GSE controller
 */
/*
void send_gse_set_vlv_cmd(uint32_t vlv_num, uint8_t vlv_state) {
	uint8_t cmd_packet[VLV_CMD_SZ];
	uint8_t cobs_cmd_packet[VLV_CMD_SZ+2];

	cmd_packet[0] = 8;  // set_vlv
	cmd_packet[1] = FLIGHT_EC_ADDR;       // origin_addr
	cmd_packet[2] = GSE_CONTROLLER_ADDR;  // target_addr
	cmd_packet[3] = 1;  // priority
	cmd_packet[4] = 1;  // num_packets
	cmd_packet[5] = 1;  // do_cobbs
	cmd_packet[6] = 0;  // checksum
	cmd_packet[7] = 0;  // checksum
	cmd_packet[8] = (SYS_MICROS >> 0) & 0xFF;    // timestamp
	cmd_packet[9] = (SYS_MICROS >> 8) & 0xFF;    // timestamp
	cmd_packet[10] = (SYS_MICROS >> 16) & 0xFF;  // timestamp
	cmd_packet[11] = (SYS_MICROS >> 24) & 0xFF;  // timestamp
	cmd_packet[12] = (vlv_num >> 0) & 0xFF;
	cmd_packet[13] = (vlv_num >> 8) & 0xFF;
	cmd_packet[14] = (vlv_num >> 16) & 0xFF;
	cmd_packet[15] = (vlv_num >> 24) & 0xFF;
	cmd_packet[16] = vlv_state;

	uint16_t packed_sz = stuff_packet(cmd_packet, cobs_cmd_packet, VLV_CMD_SZ);

	HAL_UART_Transmit(&COM_UART, cobs_cmd_packet, packed_sz, HAL_MAX_DELAY);
}
*/

#endif /* INC_BOARD_COMMANDS_H_ */
