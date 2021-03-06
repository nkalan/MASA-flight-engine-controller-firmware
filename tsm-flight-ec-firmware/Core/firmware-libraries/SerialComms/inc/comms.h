/*
 * comms.h
 *
 *  Created on: Jul 21, 2020
 *      Author: Arthur Zhang, Jack Taliercio
 */

#ifndef COMMS_H
#define COMMS_H
/*
    Since this library is not necessarily chip specific, instead of using the
    chip number as a unique prefix for global variables, we will instead use
    the prefix CLB for Comms LiBrary
*/

/* Included */
#include "stdint.h"
#include "math.h"
//#include "stm32l4xx_hal.h" // TODO change back to f4 after testing
#include "stm32f4xx_hal.h"

#include "pack_cmd_defines.h"
#include "pack_telem_defines.h"

//#ifdef PACK_CALIBRATION_DEFINES_H
#include "pack_calibration_defines.h"
//#endif

/* Global Defines */
#define PING_MAX_PACKET_SIZE        253
#define PONG_MAX_PACKET_SIZE        255
#define CLB_HEADER_SZ               12       // packet header struct size (bytes)

/* Public Function Prototypes */

// Packet Header 
typedef struct CLB_Packet_Header {
    uint8_t packet_type;        // CMD/DATA packet ID
    uint8_t origin_addr;        // origin board address
    uint8_t target_addr;        // target board address
    uint8_t priority;           // priority of packet
    uint8_t num_packets;        // number of packets in current transmission
    uint8_t do_cobbs;           // 1 to enable cobbs encoding
    uint16_t checksum;          // checksum to ensure robustness (generated)
    uint32_t timestamp;         // timestamp for data
} CLB_Packet_Header;

typedef struct CLB_send_data_info {
	UART_HandleTypeDef* uartx;
	int16_t flash_arr_sz;
	int16_t flash_arr_used;
	uint8_t *flash_arr;
} CLB_send_data_info;

enum CLB_send_data_errors {
	CLB_nominal					= 0,
	CLB_flash_buffer_overflow 	= 1
};

enum CLB_send_data_type {
	CLB_Telem = 0,
	CLB_Flash = 1
};

enum CLB_receive_data_status {
    CLB_RECEIVE_NOMINAL         = 0,
    CLB_RECEIVE_SZ_ERROR        = 1,
    CLB_RECEIVE_DAISY_TELEM     = 2,
    CLB_RECEIVE_CHECKSUM_ERROR  = 3
};

/* Telemetry Data */
uint8_t CLB_ping_packet[PING_MAX_PACKET_SIZE];   // unencoded packet (ping)
uint8_t CLB_pong_packet[PONG_MAX_PACKET_SIZE];   // encoded packet (pong)
uint8_t *CLB_buffer;                        // generic raw data buffer
uint16_t CLB_buffer_sz;
uint8_t CLB_telem_data[CLB_NUM_TELEM_ITEMS];   // autogenerated telem data buffer
CLB_Packet_Header* CLB_header;              // user specified clb header
uint8_t CLB_board_addr;
uint8_t CLB_last_cmd_received;

CLB_Packet_Header CLB_receive_header;       // private header for receive packets

//#ifdef PACK_CALIBRATION_DEFINES_H
uint8_t CLB_calibration_data[CLB_NUM_CALIBRATION_ITEMS];
//#endif

/**
    Points CLB data arr to array buffer that encode/send
    @param  buffer      <uint8_t> array of data to encode/send
    @param  buffer_sz   <uint8_t> data array size
    @param  header      <CLB_Packet_Header*> arguments necessary for encoding
                        data packet header

    Note: specifying a buffer_sz of -1 signifies to use the autogenerated 
            telems bufferinstead of arguments buffer
*/
void init_data(uint8_t *buffer, int16_t buffer_sz, CLB_Packet_Header* header);

uint8_t* return_telem_buffer(uint8_t*buffer_sz);

/**
    Sends data currently in buffer
    @param  uartx       <UART_HandleTypeDef*> uart channel from which to send
                        data
    @returns            <uint8_t> status of data transmission 0 - no error
*/
uint8_t send_data(CLB_send_data_info* info, uint8_t type);

// TODO:
uint8_t receive_data(UART_HandleTypeDef* uartx, uint8_t* buffer, uint16_t buffer_sz);

// TODO: initialize board
void init_board(uint8_t board_addr);

/* Private Function Prototypes */

uint8_t compute_packet_sz();

// TODO: do definition
void pack_telem_data(uint8_t* dst);

// TODO: do definition
void receive_packet(UART_HandleTypeDef* uartx, uint16_t sz);

// TODO: do definition
void transmit_packet(UART_HandleTypeDef* uartx, uint16_t sz);

// TODO: do definition
void pack_header(CLB_Packet_Header* header, uint8_t*header_buffer);

// TODO: do definition
void unpack_header(CLB_Packet_Header* header,uint8_t* header_buffer);

/**
 *  packs (updates) the unstuffed telem packet with new data
 *
 *  @param unstuffed	Pointer to unstuffed array that needs to be updated 
 *                      with new data 
 *
 *  Note: length must less than or equal to 
 */
void pack_packet(uint8_t *src, uint8_t *dst, uint16_t sz);

/**
 *  Stuff, using COBS https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing,
 *  an ustuffed packet in preperation for transmission
 *
 *  @param unstuffed	Pointer to unstuffed populated array
 *  @param stuffed		Pointer to unpolulated array that will be stuffed
 *  @param length		length of the unstuffed packet to be stuffed
 *
 *	@returns			Returns the length of the stuffed packet
 *  Note:
 */
uint16_t stuff_packet(const uint8_t *unstuffed, uint8_t *stuffed, uint16_t length);

/**
 *  Unstuff, assuming COBS https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing,
 *  an stuffed packet to be decoded
 *
 *  @param unstuffed	Pointer to unstuffed populated array
 *  @param stuffed		Pointer to unpolulated array that will be stuffed
 *  @param length		length of the unstuffed packet to be stuffed
 *
 *	@returns			Returns the length of the stuffed packet
 *  Note:
 */
uint16_t unstuff_packet(uint8_t *stuffed, uint8_t *unstuffed, uint16_t length);

/**
 * Computes checksum from CLB_buffer using ____ encoding
 * 
 * @returns             computed checksum
 */
uint16_t compute_checksum();

/**
 * Verifies checksum from CLB_buffer using ____ encoding
 * 
 * @param checksum      <uint16_t> checksum value
 * 
 * @returns             status of verification - 0 is good
 */
uint8_t verify_checksum(uint16_t checksum);


#endif /* INC_COMMS_H_ */
