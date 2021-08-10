/*
 * serial_data.c
 *
 *  Created on: May 18, 2021
 *      Author: natha
 */


//#include <stdio.h>
#include <string.h>
#include "serial_data.h"
#include "main.h"
#include "comms.h"
#include "constants.h"
#include "globals.h"

#include "pack_calibration_defines.h"


extern IWDG_HandleTypeDef hiwdg;


W25N01GV_Flash flash;  // Flash struct
uint8_t telem_disabled = 0;
//DmaBufferInfo buffer_info;  // DMA rx

uint8_t calibration_telem_buffer[CLB_NUM_CALIBRATION_ITEMS];


/**
 * Initialize the flash struct and chip.
 * Initialize all the DMA rx buffers.
 */
void init_serial_data(/*DmaBufferInfo* buffer_info*/) {
	// Flash
	//init_flash(&flash, &SPI_MEM, FLASH_CS_GPIO_Port, FLASH_CS_Pin);

	// DMA RX
	/*
	buffer_info->curr_circular_buffer_pos = 0;
	buffer_info->last_telem_packet_pos = 0;

	for (uint8_t i = 0; i < NUM_BUFFER_PACKETS; i++) {
		buffer_info->curr_telem_start[i] = 0;
		buffer_info->curr_telem_len[i] = 0;
	}

	for (uint16_t i = 0; i < PONG_MAX_PACKET_SIZE; i++) {
		buffer_info->telem_buffer[i] = 0;
	}
	*/
}

void update_serial_data_vars() {
    flash_mem = get_bytes_remaining(&flash);

    // Update last command received
    last_command_id = CLB_last_cmd_received;
}


/**
 * Some info doesn't change between telem and flash packet headers
 */
void init_packet_header(CLB_Packet_Header* header, uint8_t target_addr) {
	header->packet_type = 0; // default packet
	header->origin_addr = OWN_BOARD_ADDR;
	header->target_addr = target_addr;
	header->priority = 1; // medium
	header->num_packets = 1;
	header->do_cobbs = 1; // enable COBS
	header->checksum = 0;
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

// Mostly copied from press board
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
		HAL_IWDG_Refresh(&hiwdg);
	}
	reset_flash_read_pointer(&flash);
}


/**
 * Called by handle_uart_dma_rx, copies dma data to
 * circular buffer for transmission
 *
 * NOTE: for transmitting a daisy chained packet to another board ONLY
 * Not used on this board
 */
/*
void copyDataToBuffer(uint8_t* buffer, int16_t buffer_sz,
		DmaBufferInfo* buffer_info) {
    uint8_t* buffer_tail = buffer_info->circular_telem_buffer
    		+ CIRCULAR_TELEM_BUFFER_SZ;
    uint8_t* curr = buffer_info->circular_telem_buffer
    		+ buffer_info->curr_circular_buffer_pos;

    if (curr + buffer_sz >= buffer_tail) {
        uint32_t buffer_part_sz = (uint32_t) (buffer_tail - curr);
        memcpy(curr, buffer, buffer_part_sz);
        buffer_sz = buffer_sz - buffer_part_sz;
        buffer = buffer + buffer_part_sz;
        curr = buffer_info->circular_telem_buffer;
    }
    memcpy(curr, buffer, buffer_sz);
}
*/


/**
 * Called by the DMA rx interrupt callback function.
 * Receives and handles a packet from UART DMA.
 */
/*
void handle_uart_dma_rx(UART_HandleTypeDef *huart, DmaBufferInfo* buffer_info) {

    // UART Rx Complete Callback;
    // Rx Complete is called by: DMA (automatically), if it rolls over
    // and when an IDLE Interrupt occurs
    // DMA Interrupt always occurs BEFORE the idle interrupt can be fired
    // because idle detection needs at least one UART clock to detect the
    // bus is idle. So in the case, that the transmission length is one full
    // buffer lengt and the start buffer pointer is at 0, it will be also 0
    // at the end of the transmission. In this case the DMA rollover will
    // increment the RxRollover variable first and len will not be zero.

    static uint8_t RxRollover  = 0;
    static uint16_t RxBfrPos   = 0;

    // Check if it is an "Idle Interrupt"
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {

    	// clear the interrupt
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        // Rx bytes start position (= last buffer position)
        uint16_t start = RxBfrPos;

        // determine actual buffer position
        RxBfrPos = DMA_RX_BUFFER_SIZE
        		- (uint16_t)huart->hdmarx->Instance->NDTR;
        // init len with max. size
        uint16_t len = DMA_RX_BUFFER_SIZE;

        if (RxRollover < 2)  {
        	// rolled over once
            if (RxRollover) {
                if (RxBfrPos <= start) {
                	// no bytes overwritten
                	len = RxBfrPos + DMA_RX_BUFFER_SIZE - start;
                }
                else {
                	// bytes overwritten error
                    len = DMA_RX_BUFFER_SIZE + 1;
                }
            }
            else {
            	// no bytes overwritten
                len = RxBfrPos - start;
            }
        }
        else {
        	// dual rollover error
            len = DMA_RX_BUFFER_SIZE + 2;
        }

        if (len && (len <= DMA_RX_BUFFER_SIZE)) {
            uint16_t bytes_in_first_part = len;
            uint16_t bytes_in_second_part = 0;

            // if data loops in buffer
            if (RxBfrPos < start) {
                bytes_in_first_part = DMA_RX_BUFFER_SIZE - start;
                bytes_in_second_part= len - bytes_in_first_part;
            }

            // handle telem for yourself immediately
            memcpy(buffer_info->telem_buffer, buffer_info->DMA_RX_Buffer
            		+ start, bytes_in_first_part);
            memcpy(buffer_info->telem_buffer + bytes_in_first_part,
            		buffer_info->DMA_RX_Buffer, bytes_in_second_part);
            uint8_t cmd_status = receive_data(huart,
            		buffer_info->telem_buffer, len);

            // handle telem for others in buffer
            // TODO: remove this
            if (cmd_status == CLB_RECEIVE_DAISY_TELEM) {
            	buffer_info->curr_telem_start[buffer_info->last_telem_packet_pos]
											  = buffer_info->curr_circular_buffer_pos;
                copyDataToBuffer(buffer_info->telem_buffer, len, buffer_info);
                buffer_info->curr_telem_len[buffer_info->last_telem_packet_pos] = len;
                buffer_info->last_telem_packet_pos = (buffer_info->last_telem_packet_pos
                		+ 1) % NUM_BUFFER_PACKETS;
            }
        } else {
            // buffer overflow error:
        	// TODO
            //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }

        RxRollover = 0;  // reset the Rollover variable
    } else {
        // no idle flag? --> DMA rollover occurred
        RxRollover++;       // increment Rollover Counter
    }
}
*/

void send_calibration_data() {
	CLB_Packet_Header cal_header;

	cal_header.packet_type = 2; // calibration packet
	cal_header.origin_addr = OWN_BOARD_ADDR;
	cal_header.target_addr = SERVER_ADDR;
	cal_header.priority = 1; // medium
	cal_header.num_packets = 1;
	cal_header.do_cobbs = 1; // enable COBS
	cal_header.checksum = 0;
	cal_header.timestamp = SYS_MICROS;

	pack_calibration_data(calibration_telem_buffer);
    init_data(calibration_telem_buffer, CLB_NUM_CALIBRATION_ITEMS, &cal_header);  // Comms library (tx, so no buffer)

    CLB_send_data_info info;
    info.uartx = &COM_UART;
    send_data(&info, CLB_Telem);
}
