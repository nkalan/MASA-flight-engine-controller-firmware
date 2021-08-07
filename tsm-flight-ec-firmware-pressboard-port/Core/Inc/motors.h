/*
 * motors.h
 *
 *  Created on: Aug 6, 2021
 *      Author: natha
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdint.h>


// Motor Datasheet
#define degPerStep  (float) 0.35
#define stepPerDeg  (float) 2.85714


typedef struct Stepper_Pinfo {
	GPIO_TypeDef* motor_ports[4];
	uint16_t motor_pins[4];
	uint32_t pwm_channel[2];
} Stepper_Pinfo;


typedef struct Stepper_Ramping {
    // base variables
    volatile uint16_t SPS_tim_reg;
    volatile uint16_t SPS;
    volatile uint16_t SPS_target;
    uint32_t tim_osc;
    uint16_t acc_res;
    volatile uint16_t acc_step;

    // calculated parameters
    volatile uint16_t num_steps;
    volatile uint16_t steps_to_stop;
    volatile uint16_t curr_step;
} Stepper_Ramping;


Stepper_Pinfo stepper_pinfo[2]; // information about stepper motor pin outs


uint8_t isMotorAtPos(float target, uint8_t tank_num);

void handleMotorStepping(uint8_t motor_num);

void turn_stepper_motor(TIM_HandleTypeDef *htimx, uint8_t motor_num, int8_t direction, uint8_t step_stage);

void moveMotorToPos(float deg, uint8_t motor_num);

// Not used
void computeStepperRamp(Stepper_Ramping* stepper);

// Not used
void changeStepperPeriod(TIM_HandleTypeDef *htimx, Stepper_Ramping* stepper);


#endif /* INC_MOTORS_H_ */
