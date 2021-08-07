/*
 * motors.c
 *
 *  Created on: Aug 6, 2021
 *      Author: natha
 */


#include "motors.h"
#include "constants.h"


volatile float curPos[NUM_TANKS];
//float Setpoint[NUM_TANKS]; useless
volatile int8_t curDir[NUM_TANKS];  // init as 0 to force correct startup direction selection
volatile uint8_t manual_stepper_pos_override[NUM_TANKS];
volatile float targetPos[NUM_TANKS];
volatile float curPos[NUM_TANKS];

TIM_HandleTypeDef* motor_pwm_tim[NUM_TANKS] = {&htim2, &htim3};
TIM_HandleTypeDef* motor_step_tim[NUM_TANKS] = {&htim13, &htim6};

float posErrorMargin = degPerStep; // arbitrary error margin change later


void computeStepperRamp(Stepper_Ramping* stepper) {
    uint16_t step = stepper->curr_step++;
    uint16_t steps_rem = stepper->steps_to_stop; // slowdown phase
    uint16_t steps_hold = stepper->num_steps - steps_rem; // startup phase

    if (step > steps_rem && stepper->SPS >= 1000) { // deceleration phase
        stepper->SPS-=stepper->acc_res;
    } else if (step < steps_hold &&
            stepper->SPS < stepper->SPS_target) { // acceleration phase
        stepper->SPS+=stepper->acc_res;
    }

    stepper->SPS_tim_reg = (uint16_t)(stepper->tim_osc / stepper->SPS);
}

void changeStepperPeriod(TIM_HandleTypeDef *htimx, Stepper_Ramping* stepper) {
    uint16_t new_period = (uint16_t)(stepper->SPS_tim_reg);

    HAL_TIM_Base_Stop_IT(htimx);
    // htimx->Instance =  no need to change instance
    htimx->Init.Prescaler = 15;
    htimx->Init.CounterMode = TIM_COUNTERMODE_UP;
    htimx->Init.Period = new_period;
    htimx->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htimx->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(htimx);

    HAL_TIM_Base_Start_IT(htimx);
}

uint8_t isMotorAtPos(float target, uint8_t tank_num) {
    return fabs(target-curPos[tank_num]) < posErrorMargin;
}

static inline int8_t normalizeMotorStep(int8_t step_num) {
    if(step_num == 4) {
        return 0;
    } else if (step_num == -1) {
        return 3;// reset step stage if reached
    }
    return step_num;
}


void handleMotorStepping(uint8_t motor_num) {
    // Stepper motor linear ramp variables
    static int8_t step_stage[NUM_TANKS] = { 0, 0 }; // polarity of stepper motor coil
    static int8_t prevDir[NUM_TANKS] = {0};

    if (prevDir[motor_num]==0) {
        prevDir[motor_num] = curDir[motor_num];
    }

    // Step the motor if in running or override
    if (!isMotorAtPos(targetPos[motor_num], motor_num)) {

        if (prevDir[motor_num] != curDir[motor_num]) {
            if (prevDir[motor_num] == -1) {
                ++step_stage[motor_num];
            } else if (prevDir[motor_num] == 1) {
                --step_stage[motor_num];
            }
        }

        step_stage[motor_num] = normalizeMotorStep(step_stage[motor_num]);

        turn_stepper_motor(motor_pwm_tim[motor_num], motor_num,
                           curDir[motor_num], step_stage[motor_num]);

        step_stage[motor_num] += curDir[motor_num];
        step_stage[motor_num] = normalizeMotorStep(step_stage[motor_num]);
        prevDir[motor_num] = curDir[motor_num];

        // uncomment code below for dynamic stepping
//            if (stepper.acc_step == stepper.acc_res) {
//                computeStepperRamp(&stepper);
//                changeStepperPeriod(&htim6, &stepper);
//                stepper.acc_step = 0;
//            } else {
//                ++stepper.acc_step;
//            } // ensures SPSPS is followed properly
        curPos[motor_num] = curPos[motor_num] + degPerStep*curDir[motor_num];

        /* Update telem packet motor info */
        mtr_pos[motor_num] = curPos[motor_num]; // for motor 1
        mtr_vel[motor_num] = (__HAL_TIM_GET_AUTORELOAD(
                             motor_step_tim[motor_num])+1) * curDir[motor_num]; // in SPS
    }

    if (isMotorAtPos(targetPos[motor_num], motor_num)) {
        // auto disable override once position is reached
        HAL_TIM_PWM_Stop(motor_pwm_tim[motor_num],
                         stepper_pinfo[motor_num].pwm_channel[0]);
        HAL_TIM_PWM_Stop(motor_pwm_tim[motor_num],
                         stepper_pinfo[motor_num].pwm_channel[1]);
        if (manual_stepper_pos_override[motor_num]) {
            manual_stepper_pos_override[motor_num] = 0;
        }
        mtr_vel[motor_num] = 0;
        // reset stepper linear ramping variables
        // stepper.num_steps = stepper.curr_step = 0;
    }
}

void turn_stepper_motor(TIM_HandleTypeDef *htimx, uint8_t motor_num, int8_t direction, uint8_t step_stage) {
    // A+
    // B+
    // A-
    // B-
    // For VNH7070AS chip, INA = clockwise, INB = counterclockwise (actually not really)
    // reset all gpio logic before changing step
    // Note: this only works for motor 0 currently 1 is CW, -1 is CCW
	GPIO_TypeDef * mtrx_portA0 = stepper_pinfo[motor_num].motor_ports[0];
	GPIO_TypeDef * mtrx_portA1 = stepper_pinfo[motor_num].motor_ports[1];
	GPIO_TypeDef * mtrx_portB0 = stepper_pinfo[motor_num].motor_ports[2];
	GPIO_TypeDef * mtrx_portB1 = stepper_pinfo[motor_num].motor_ports[3];
	uint16_t mtrx_pinA0 = stepper_pinfo[motor_num].motor_pins[0];
	uint16_t mtrx_pinA1 = stepper_pinfo[motor_num].motor_pins[1];
	uint16_t mtrx_pinB0 = stepper_pinfo[motor_num].motor_pins[2];
	uint16_t mtrx_pinB1 = stepper_pinfo[motor_num].motor_pins[3];
	uint32_t pwm_channela = stepper_pinfo[motor_num].pwm_channel[0];
	uint32_t pwm_channelb = stepper_pinfo[motor_num].pwm_channel[1];

    // reset all motor logic inputs low and speed to 0
    HAL_TIM_PWM_Stop(htimx, pwm_channela);
    HAL_TIM_PWM_Stop(htimx, pwm_channelb);
    // stepper motor channels
    switch(step_stage) { // TODO figure out steps tmrw
        case 0: {   // A0 high B0 low
        	HAL_GPIO_WritePin(mtrx_portA1, mtrx_pinA1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(mtrx_portB1, mtrx_pinB1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(mtrx_portA0, mtrx_pinA0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(mtrx_portB0, mtrx_pinB0, GPIO_PIN_SET);
            break;
        } case 1: {
        	HAL_GPIO_WritePin(mtrx_portB1, mtrx_pinB1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(mtrx_portA0, mtrx_pinA0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(mtrx_portB0, mtrx_pinB0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(mtrx_portA1, mtrx_pinA1, GPIO_PIN_SET);
            break;
        } case 2: {
        	HAL_GPIO_WritePin(mtrx_portA0, mtrx_pinA0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(mtrx_portB0, mtrx_pinB0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(mtrx_portA1, mtrx_pinA1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(mtrx_portB1, mtrx_pinB1, GPIO_PIN_SET);
            break;  // swap which input is high/low for for negative polarity
        } case 3: {
        	HAL_GPIO_WritePin(mtrx_portB0, mtrx_pinB0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(mtrx_portA1, mtrx_pinA1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(mtrx_portB1, mtrx_pinB1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(mtrx_portA0, mtrx_pinA0, GPIO_PIN_SET);
            break;
        }
    }
    HAL_TIM_PWM_Start(htimx, pwm_channela); // start pwm on channels 1 and 2 for stepper
    HAL_TIM_PWM_Start(htimx, pwm_channelb);
}

void moveMotorToPos(float deg, uint8_t motor_num) {
	manual_stepper_pos_override[motor_num] = 1;
	targetPos[motor_num] = deg; // position given in deg
	curDir[motor_num] = (curPos[motor_num] < targetPos[motor_num]) ? 1 : -1; // CCW facing the motor
	mtr_set[motor_num] = deg; // save new motor position setpoint
}
