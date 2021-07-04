/*
 * serial_data.c
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */


#include <serial_data.h>
#include "comms.h"
#include "W25N01GV.h"
#include "constants.h"


W25N01GV_Flash flash;
uint8_t telem_disabled = 0;

//uint8_t telem_packet[255];

/**
 * Some info doesn't change between telem and flash packet headers
 */
void init_packet_header(CLB_Packet_Header* header, uint8_t target_addr) {
	header->packet_type = 0; // default packet
	header->origin_addr = FLIGHT_EC_ADDR;
	header->target_addr = target_addr;
	header->priority = 1; // medium
	header->do_cobbs = 1; // enable COBS
	header->timestamp = SYS_MICROS;
}

void send_telem_packet(uint8_t target_addr) {
	CLB_Packet_Header telem_header;
	init_packet_header(&telem_header, target_addr);
    init_data(NULL, -1, &telem_header);  // Comms library (tx, so no buffer)

    CLB_send_data_info info;
    info.uartx = &COM_UART;
    send_data(&info, CLB_Telem);
}

void save_flash_packet() {

	CLB_Packet_Header flash_header;
	init_packet_header(&flash_header, SERVER_ADDR);
    init_data(NULL, -1, &flash_header);   // Comms library (tx, so no buffer)

    // packs data to flash
    uint8_t buffer[253] = {0};
    CLB_send_data_info info;
    info.flash_arr_used = 0;
    info.flash_arr_sz = 253; // arbitrary
    info.flash_arr = buffer;
    send_data(&info, CLB_Flash);

    uint8_t buffer_sz = info.flash_arr_used;

    // Write to Flash
    write_to_flash(&flash, buffer, buffer_sz);
}

// Copied from press board, but without passing in a flash struct pointer
void transmit_flash_data() {

	// Ensure flash is flushed
	finish_flash_write(&flash);

	uint32_t page = 0;
	uint32_t end_page = flash.current_page+1;
	if (flash.next_free_column == 0) // if the last page is completely empty, ignore it
		end_page--;
	uint8_t read_buffer[W25N01GV_BYTES_PER_PAGE];  // W25N01GV_BYTES_PER_PAGE == 2048 == 2KB
	reset_flash_read_pointer(&flash);
	while (page < end_page) {
	    read_next_2KB_from_flash(&flash, read_buffer);
	    HAL_UART_Transmit(&COM_UART, read_buffer, W25N01GV_BYTES_PER_PAGE, HAL_MAX_DELAY);
	    ++page;
		//HAL_IWDG_Refresh(&hiwdg);  TODO: re enable this
	}
	reset_flash_read_pointer(&flash);
}


