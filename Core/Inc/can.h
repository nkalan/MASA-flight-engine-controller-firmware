/*
 * can.h
 *
 *  Created on: May 23, 2021
 *      Author: natha
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f4xx_hal_can.h"


void init_CAN(CAN_HandleTypeDef* hcan);

void transmit_CAN_msg(CAN_HandleTypeDef* hcan, uint8_t tx_msg, uint16_t tx_msg_sz);

void receive_CAN_msg(CAN_HandleTypeDef* hcan, uint8_t rx_msg, uint16_t rx_msg_sz);

#endif /* INC_CAN_H_ */
