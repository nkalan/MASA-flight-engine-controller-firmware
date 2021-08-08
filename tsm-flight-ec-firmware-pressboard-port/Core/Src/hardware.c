#include "hardware.h"
#include "math.h"
#include "calibrations.h"

uint16_t adc_counts[3][16];  // array for receiving adc conversions
GPIO_MAX11131_Pinfo adc_pins[3];
MAX31855_Pinfo tc_pins[5];

TIM_HandleTypeDef* motor_pwm_tim[NUM_TANKS] = {&htim2, &htim3};
TIM_HandleTypeDef* motor_step_tim[NUM_TANKS] = {&htim13, &htim6};

volatile uint8_t manual_stepper_pos_override[NUM_TANKS]     = {0};

volatile float targetPos[NUM_TANKS] = {0, 0};
volatile float curPos[NUM_TANKS] = {0, 0};
extern TIM_HandleTypeDef htim5;

float maxPos = 7 * 360 - 60;

float posErrorMargin = degPerStep; // arbitrary error margin change later

Stepper_Ramping stepper;

volatile int8_t curDir[NUM_TANKS] = {0};  // init as 0 to force correct startup direction selection
volatile uint8_t manual_stepper_pos_override[NUM_TANKS];


// pot ambient
//float pot_ambients[4] = {0};

// ADC calibrations
static ADC_Cal adc_calibrations[3];


Potentiometer_Cal pot_cal_info;
Stepper_Pinfo stepper_pinfo[2]; // information about stepper motor pin outs


void init_hardware() {

    initAdcs(&hspi1, adc_pins);   // initialize adcs

    // init stepper motor pins
    stepper_pinfo[0].motor_ports[0] = mtr0_inA0_GPIO_Port;
    stepper_pinfo[0].motor_ports[1] = mtr0_inA1_GPIO_Port;
    stepper_pinfo[0].motor_ports[2] = mtr0_inB0_GPIO_Port;
    stepper_pinfo[0].motor_ports[3] = mtr0_inB1_GPIO_Port;
    stepper_pinfo[0].motor_pins[0] = mtr0_inA0_Pin;
    stepper_pinfo[0].motor_pins[1] = mtr0_inA1_Pin;
    stepper_pinfo[0].motor_pins[2] = mtr0_inB0_Pin;
    stepper_pinfo[0].motor_pins[3] = mtr0_inB1_Pin;
    stepper_pinfo[0].pwm_channel[0] = TIM_CHANNEL_1;
    stepper_pinfo[0].pwm_channel[1] = TIM_CHANNEL_2;

    stepper_pinfo[1].motor_ports[0] = mtr1_inA0_GPIO_Port;
    stepper_pinfo[1].motor_ports[1] = mtr1_inA1_GPIO_Port;
    stepper_pinfo[1].motor_ports[2] = mtr1_inB0_GPIO_Port;
    stepper_pinfo[1].motor_ports[3] = mtr1_inB1_GPIO_Port;
    stepper_pinfo[1].motor_pins[0] = mtr1_inA0_Pin;
    stepper_pinfo[1].motor_pins[1] = mtr1_inA1_Pin;
    stepper_pinfo[1].motor_pins[2] = mtr1_inB0_Pin;
    stepper_pinfo[1].motor_pins[3] = mtr1_inB1_Pin;
    stepper_pinfo[1].pwm_channel[0] = TIM_CHANNEL_1;
    stepper_pinfo[1].pwm_channel[1] = TIM_CHANNEL_2;

    tc_pins[0].MAX31855_CS_PORT = tc0_cs_GPIO_Port;
    tc_pins[1].MAX31855_CS_PORT = tc1_cs_GPIO_Port;
    tc_pins[2].MAX31855_CS_PORT = tc2_cs_GPIO_Port;
    tc_pins[3].MAX31855_CS_PORT = tc3_cs_GPIO_Port;
    tc_pins[4].MAX31855_CS_PORT = tc4_cs_GPIO_Port;
    tc_pins[0].MAX31855_CS_ADDR = tc0_cs_Pin;
    tc_pins[1].MAX31855_CS_ADDR = tc1_cs_Pin;
    tc_pins[2].MAX31855_CS_ADDR = tc2_cs_Pin;
    tc_pins[3].MAX31855_CS_ADDR = tc3_cs_Pin;
    tc_pins[4].MAX31855_CS_ADDR = tc4_cs_Pin;

    // Potentiometer Calibrations
    pot_cal_info.slope = (EPOT_DMAX - EPOT_DMIN)*(EPOT_RES_POT+2*EPOT_RES_LEAD)
                            /(EPOT_MAX_COUNTS*(EPOT_RES_POT - 2.0*EPOT_RES_POT_MIN));
    pot_cal_info.offset = ((EPOT_MAX_COUNTS*(EPOT_RES_POT_MIN + EPOT_RES_LEAD))
                            / (EPOT_RES_POT + 2.0*EPOT_RES_LEAD));

    // Load constants from flash on board startup
    //is_flash_constant_good = read_constants_from_flash(&flash);

    // Set autopress pressure check ranges
    //set_autopress_pressures();

    // Reamining time in current state
    //state_rem_duration = 0;

    // IWDG status LED
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); // turns off when something has gone wrong
}

// private helpers
static void setValveHelper(uint32_t vlv_num, GPIO_PinState gpio_state);

void initAdcs(SPI_HandleTypeDef* hspix, GPIO_MAX11131_Pinfo *adc_pins) {
    adc_pins[0].MAX11131_CS_PORT       = adc0_cs_GPIO_Port;
    adc_pins[0].MAX11131_EOC_PORT      = adc0_eoc_GPIO_Port;
    adc_pins[0].MAX11131_CNVST_PORT    = adc0_cnvst_GPIO_Port;
    adc_pins[0].MAX11131_CS_ADDR       = adc0_cs_Pin;
    adc_pins[0].MAX11131_EOC_ADDR      = adc0_eoc_Pin;
    adc_pins[0].MAX11131_CNVST_ADDR    = adc0_cnvst_Pin;
    adc_pins[1].MAX11131_CS_PORT       = adc1_cs_GPIO_Port;
    adc_pins[1].MAX11131_EOC_PORT      = adc1_eoc_GPIO_Port;
    adc_pins[1].MAX11131_CNVST_PORT    = adc1_cnvst_GPIO_Port;
    adc_pins[1].MAX11131_CS_ADDR       = adc1_cs_Pin;
    adc_pins[1].MAX11131_EOC_ADDR      = adc1_eoc_Pin;
    adc_pins[1].MAX11131_CNVST_ADDR    = adc1_cnvst_Pin;
    adc_pins[2].MAX11131_CS_PORT       = adc2_cs_GPIO_Port;
    adc_pins[2].MAX11131_EOC_PORT      = adc2_eoc_GPIO_Port;
    adc_pins[2].MAX11131_CNVST_PORT    = adc2_cnvst_GPIO_Port;
    adc_pins[2].MAX11131_CS_ADDR       = adc2_cs_Pin;
    adc_pins[2].MAX11131_EOC_ADDR      = adc2_eoc_Pin;
    adc_pins[2].MAX11131_CNVST_ADDR    = adc2_cnvst_Pin;
    init_adc(hspix, &adc_pins[0]);
    init_adc(hspix, &adc_pins[1]);
    init_adc(hspix, &adc_pins[2]);

    // Initialize ADC offset and slopes, defaults to 0 and 1 for offset and slopes
    for (uint8_t i = 0; i < 3; ++i) {
    	for (uint16_t ch = 0; ch < 16; ++ch) {
    		adc_calibrations[i].offset[ch] 	= 0;
    		adc_calibrations[i].slope[ch] 	= 1;
    	}
    }
}

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
    if ((STATE==Hotfire || STATE==Startup || STATE==Post || STATE==Abort
            || manual_stepper_pos_override[motor_num])
            && !isMotorAtPos(targetPos[motor_num], motor_num)) {

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

/*
void save_telem_to_flash(W25N01GV_Flash* flash) {
    // Prepare Telem Data For Flashing
    CLB_Packet_Header header;
    header.packet_type = 0; // default packet
	header.origin_addr = 3;
	header.target_addr = 7; // computer
    header.priority = 1; // medium
    header.do_cobbs = 1; // enable cobs
    header.timestamp= SYS_MICROS;
    init_data(NULL, -1, &header);   // pack all telem data

    // packs data to flash
    uint8_t buffer[253] = {0};
    CLB_send_data_info info;
    info.flash_arr_used = 0; // TODO: move this to auto init function
    info.flash_arr_sz = 253; // arbitrary
    info.flash_arr = buffer;
    send_data(&info, CLB_Flash);

    uint8_t buffer_sz = info.flash_arr_used;

    // Write to Flash
    write_to_flash(flash, buffer, buffer_sz);
}
*/

/*
void send_telem_from_flash(W25N01GV_Flash* flash, UART_HandleTypeDef* huartx) {
	// Ensure flash is flushed
	finish_flash_write(flash);

	uint32_t page = 0;
	uint32_t end_page = flash->current_page+1;
	if (flash->next_free_column == 0) // if the last page is completely empty, ignore it
		end_page--;
	uint8_t read_buffer[W25N01GV_BYTES_PER_PAGE];  // W25N01GV_BYTES_PER_PAGE == 2048 == 2KB
	reset_flash_read_pointer(flash);
	while (page < end_page) {
	    read_next_2KB_from_flash(flash, read_buffer);
	    HAL_UART_Transmit(huartx, read_buffer, W25N01GV_BYTES_PER_PAGE, HAL_MAX_DELAY);
	    ++page;
		// Refresh watchdog timer because this may take awhile
		HAL_IWDG_Refresh(&hiwdg);
	}
	reset_flash_read_pointer(flash);
}
*/

/*
void write_constants_to_flash(W25N01GV_Flash* flash) {
    flash_constants_buffer[FLASH_CONSTANT_PARITY_BIT]= 0; // 0 means written
    flash_constants_buffer[FLASH_CONSTANT_STATE_POS]= STATE;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS]  =
                                            ((uint16_t) (mtr_ki[0]*100)) >> 0;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+1]=
                                            ((uint16_t) (mtr_ki[0]*100)) >> 8;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+2]=
                                            ((uint16_t) (mtr_ki[1]*100)) >> 0;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+3]=
                                            ((uint16_t) (mtr_ki[1]*100)) >> 8;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+4]=
                                            ((uint16_t) (mtr_kd[0]*100)) >> 0;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+5]=
                                            ((uint16_t) (mtr_kd[0]*100)) >> 8;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+6]=
                                            ((uint16_t) (mtr_kd[1]*100)) >> 0;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+7]=
                                            ((uint16_t) (mtr_kd[1]*100)) >> 8;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+8]=
                                            ((uint16_t) (mtr_kp[0]*100)) >> 0;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+9]=
                                            ((uint16_t) (mtr_kp[0]*100)) >> 8;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+10]=
                                            ((uint16_t) (mtr_kp[1]*100)) >> 0;
    flash_constants_buffer[FLASH_CONSTANT_PID_POS+11]=
                                            ((uint16_t) (mtr_kp[1]*100)) >> 8;

    for (uint8_t i = 0; i < NUM_TANKS; ++i) {
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+i*4] =
                                        ((int32_t) (tank_tar_pres[i]*100)) >> 0;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+i*4+1] =
                                        ((int32_t) (tank_tar_pres[i]*100)) >> 8;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+i*4+2] =
                                        ((int32_t) (tank_tar_pres[i]*100)) >> 16;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+i*4+3] =
                                        ((int32_t) (tank_tar_pres[i]*100)) >> 24;

        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+i*4] =
                                        ((int32_t) (tank_high_pres[i]*100)) >> 0;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+i*4+1] =
                                        ((int32_t) (tank_high_pres[i]*100)) >> 8;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+i*4+2] =
                                        ((int32_t) (tank_high_pres[i]*100)) >> 16;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+i*4+3] =
                                        ((int32_t) (tank_high_pres[i]*100)) >> 24;

        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+i*4] =
                                        ((int32_t) (tank_low_pres[i]*100)) >> 0;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+i*4+1] =
                                        ((int32_t) (tank_low_pres[i]*100)) >> 8;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+i*4+2] =
                                        ((int32_t) (tank_low_pres[i]*100)) >> 16;
        flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+i*4+3] =
                                        ((int32_t) (tank_low_pres[i]*100)) >> 24;
    }

    for (uint8_t i = 0; i < NUM_PTS; ++i) {
        flash_constants_buffer[FLASH_CONSTANT_PT_AMBIENTS_POS+4*i] =
                                                ((int32_t) (pt_ambients[i]*10)) >> 0;
        flash_constants_buffer[FLASH_CONSTANT_PT_AMBIENTS_POS+4*i+1] =
                                                ((int32_t) (pt_ambients[i]*10)) >> 8;
        flash_constants_buffer[FLASH_CONSTANT_PT_AMBIENTS_POS+4*i+2] =
												((int32_t) (pt_ambients[i]*10)) >> 16;
		flash_constants_buffer[FLASH_CONSTANT_PT_AMBIENTS_POS+4*i+3] =
												((int32_t) (pt_ambients[i]*10)) >> 24;
    }

    for (uint8_t i = 0; i < NUM_POTS; ++i) {
        flash_constants_buffer[FLASH_CONSTANT_POT_AMBIENTS_POS+4*i+0] =
                                                ((int32_t) (pot_ambients[i]*10)) >> 0;
        flash_constants_buffer[FLASH_CONSTANT_POT_AMBIENTS_POS+4*i+1] =
                                                ((int32_t) (pot_ambients[i]*10)) >> 8;
        flash_constants_buffer[FLASH_CONSTANT_POT_AMBIENTS_POS+4*i+2] =
                                                ((int32_t) (pot_ambients[i]*10)) >> 16;
        flash_constants_buffer[FLASH_CONSTANT_POT_AMBIENTS_POS+4*i+3] =
                                                ((int32_t) (pot_ambients[i]*10)) >> 24;
    }

    flash_constants_buffer[FLASH_CONSTANT_TANK_ENABLE] =
                                            ((uint8_t) is_tank_enabled[0]);
    flash_constants_buffer[FLASH_CONSTANT_TANK_ENABLE+1] =
                                            ((uint8_t) is_tank_enabled[1]);

    erase_reserved_flash_pages(flash);
    write_reserved_flash_page(flash, FLASH_CONSTANTS_PAGE_NUM,
                               flash_constants_buffer, FLASH_CONSTANTS_BUFFER_SZ);
}

uint8_t read_constants_from_flash(W25N01GV_Flash* flash) {
    read_reserved_flash_page(flash, FLASH_CONSTANTS_PAGE_NUM,
                             flash_constants_buffer, FLASH_CONSTANTS_BUFFER_SZ);

    if (flash_constants_buffer[FLASH_CONSTANT_PARITY_BIT]==0) {
        mtr_ki[0] = ((flash_constants_buffer[FLASH_CONSTANT_PID_POS+1]<<8)|
                        flash_constants_buffer[FLASH_CONSTANT_PID_POS])/100.0f;
        mtr_ki[1] = ((flash_constants_buffer[FLASH_CONSTANT_PID_POS+3]<<8)|
                        flash_constants_buffer[FLASH_CONSTANT_PID_POS+2])/100.0f;
        mtr_kd[0] = ((flash_constants_buffer[FLASH_CONSTANT_PID_POS+5]<<8)|
                        flash_constants_buffer[FLASH_CONSTANT_PID_POS+4])/100.0f;
        mtr_kd[1] = ((flash_constants_buffer[FLASH_CONSTANT_PID_POS+7]<<8)|
                        flash_constants_buffer[FLASH_CONSTANT_PID_POS+6])/100.0f;
        mtr_kp[0] = ((flash_constants_buffer[FLASH_CONSTANT_PID_POS+9]<<8)|
                        flash_constants_buffer[FLASH_CONSTANT_PID_POS+8])/100.0f;
        mtr_kp[1] = ((flash_constants_buffer[FLASH_CONSTANT_PID_POS+11]<<8)|
                        flash_constants_buffer[FLASH_CONSTANT_PID_POS+10])/100.0f;

        tank_tar_pres[0] = (
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+3]<<24)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+2]<<16)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+1]<<8)|
                flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS])/100.0f;
        tank_tar_pres[1] = (
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+7]<<24)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+6]<<16)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+5]<<8)|
                flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_TAR_PRES_POS+4])/100.0f;
        Setpoint[0] = tank_tar_pres[0];
        Setpoint[1] = tank_tar_pres[1];

        tank_high_pres[0] = (
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+3]<<24)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+2]<<16)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+1]<<8)|
                flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS])/100.0f;
        tank_high_pres[1] = (
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+7]<<24)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+6]<<16)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+5]<<8)|
                flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_HIGH_PRES_POS+4])/100.0f;

        tank_low_pres[0] = (
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+3]<<24)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+2]<<16)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+1]<<8)|
                flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS])/100.0f;
        tank_low_pres[1] = (
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+7]<<24)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+6]<<16)|
                (flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+5]<<8)|
                flash_constants_buffer[FLASH_CONSTANT_CTL_SOL_LOW_PRES_POS+4])/100.0f;

        for (uint8_t i = 0; i < NUM_PTS; ++i) {
            pt_ambients[i] =
                ((flash_constants_buffer[FLASH_CONSTANT_PT_AMBIENTS_POS+4*i+3]<<24)|
				(flash_constants_buffer[FLASH_CONSTANT_PT_AMBIENTS_POS+4*i+2]<<16)|
                (flash_constants_buffer[FLASH_CONSTANT_PT_AMBIENTS_POS+4*i+1]<<8)|
                flash_constants_buffer[FLASH_CONSTANT_PT_AMBIENTS_POS+4*i+0])/10.0f;
        }

        for (uint8_t i = 0; i < NUM_POTS; ++i) {
            pot_ambients[i] =
            ((flash_constants_buffer[FLASH_CONSTANT_POT_AMBIENTS_POS+4*i+3]<<24)|
             (flash_constants_buffer[FLASH_CONSTANT_POT_AMBIENTS_POS+4*i+2]<<16)|
             (flash_constants_buffer[FLASH_CONSTANT_POT_AMBIENTS_POS+4*i+1]<<8)|
             flash_constants_buffer[FLASH_CONSTANT_POT_AMBIENTS_POS+4*i+0])/10.0f;
        }

        is_tank_enabled[0] = flash_constants_buffer[FLASH_CONSTANT_TANK_ENABLE];
        is_tank_enabled[1] = flash_constants_buffer[FLASH_CONSTANT_TANK_ENABLE+1];
        return 1;
    } // only reload flash constant variables if flash constants have been written to

    return 0;
}

void send_telem_packet(UART_HandleTypeDef* huartx, CLB_Packet_Header* header, uint8_t* buffer, int32_t buffer_sz) {
    if (buffer_sz == -1) {
    	CLB_Packet_Header default_header;
    	default_header.packet_type = 0; // default packet
    	default_header.origin_addr = 3;
    	default_header.target_addr = 7; // computer
    	default_header.priority = 1; // medium
    	default_header.do_cobbs = 1; // disable cobbs
    	default_header.timestamp= SYS_MICROS;
        init_data(buffer, buffer_sz, &default_header);
    } else {
        // must configure buffer stream and packet header prior
		init_data(buffer, buffer_sz, header);
    }

    // send telem data TODO: if error status returned
    CLB_send_data_info info;
    info.uartx = huartx;
    send_data(&info, CLB_Telem);
}
*/

void readAdcs(SPI_HandleTypeDef* hspix, GPIO_MAX11131_Pinfo* adc_pins, uint16_t (*adc_counts)[16]) {
    for (int i = 0; i < 3; ++i) {
        read_adc(hspix, &adc_pins[i], adc_counts[i]);
        for (uint8_t ch = 0; ch < 16; ++ch) {
			adc_counts[i][ch] = adc_counts[i][ch]*adc_calibrations[i].slope[ch] + adc_calibrations[i].offset[ch];
        } // apply adc offset and slope to raw counts
    }
}

void setValve(uint32_t vlv_num, uint8_t state) {
    GPIO_PinState gpio_state = (state == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    uint32_t vlv_bit = 1<<vlv_num;
    uint32_t vlv_value = state<<vlv_num;
	setValveHelper(vlv_num, gpio_state);

    // set valves states variable
    valve_states &= ~vlv_bit;
    valve_states |= vlv_value;
}

static void setValveHelper(uint32_t vlv_num, GPIO_PinState gpio_state) {
	 switch(vlv_num) {
	    case 0: {
	        HAL_GPIO_WritePin(en_vlv0_GPIO_Port, en_vlv0_Pin, gpio_state);
	        break;
	    } case 1: {
	        HAL_GPIO_WritePin(en_vlv1_GPIO_Port, en_vlv1_Pin, gpio_state);
	        break;
	    } case 2: {
	        HAL_GPIO_WritePin(en_vlv2_GPIO_Port, en_vlv2_Pin, gpio_state);
	        break;
	    } case 3: {
	        HAL_GPIO_WritePin(en_vlv3_GPIO_Port, en_vlv3_Pin, gpio_state);
	        break;
	    } case 4: {
	        HAL_GPIO_WritePin(en_vlv4_GPIO_Port, en_vlv4_Pin, gpio_state);
	        break;
	    } case 5: {
	        HAL_GPIO_WritePin(en_vlv5_GPIO_Port, en_vlv5_Pin, gpio_state);
	        break;
	    } case 6: {
	        HAL_GPIO_WritePin(en_vlv6_GPIO_Port, en_vlv6_Pin, gpio_state);
	        break;
	    } case 7: {
	        HAL_GPIO_WritePin(en_vlv7_GPIO_Port, en_vlv7_Pin, gpio_state);
	        break;
	    } case 8: {
	        HAL_GPIO_WritePin(en_vlv8_GPIO_Port, en_vlv8_Pin, gpio_state);
	        break;
	    }
	}
}

void readThermocouples(SPI_HandleTypeDef* hspix, MAX31855_Pinfo *tc_pins,
                                                            uint8_t tc_num) {
    for (uint8_t i = 0; i < tc_num; ++i) {
    	tc[i] = read_tc(hspix, &tc_pins[i]);
    }
}

void updatePeripherals(uint16_t (*adc_counts)[16]) {
    // Programmer defined, transfer adc readings to correct memory arrays based on schematic

    /* Load vlv voltages */
    ivlv[0] = vlvCountsToVolts(adc_counts[0][0]);
    ivlv[1] = vlvCountsToVolts(adc_counts[1][10]);
    ivlv[2] = vlvCountsToVolts(adc_counts[1][8]);
    ivlv[3] = vlvCountsToVolts(adc_counts[1][6]);
    ivlv[4] = vlvCountsToVolts(adc_counts[1][4]);
    ivlv[5] = vlvCountsToVolts(adc_counts[1][2]);
    ivlv[6] = vlvCountsToVolts(adc_counts[1][0]);
    ivlv[7] = vlvCountsToVolts(adc_counts[0][11]);
    ivlv[8] = vlvCountsToVolts(adc_counts[0][13]);

    /* Load vlv currents */
    evlv[0] = vlvCountsToAmps(adc_counts[0][1]);
    evlv[1] = vlvCountsToAmps(adc_counts[1][11]);
    evlv[2] = vlvCountsToAmps(adc_counts[1][9]);
    evlv[3] = vlvCountsToAmps(adc_counts[1][7]);
    evlv[4] = vlvCountsToAmps(adc_counts[1][5]);
    evlv[5] = vlvCountsToAmps(adc_counts[1][3]);
    evlv[6] = vlvCountsToAmps(adc_counts[1][1]);
    evlv[7] = vlvCountsToAmps(adc_counts[0][10]);
    evlv[8] = vlvCountsToAmps(adc_counts[0][12]);

    /* Load pressure transducer voltages */
    for (int8_t i = 7; i >= 2; --i) {
        uint8_t channel = 7-i;
        pressure[channel] = pt_counts_to_psi( channel, adc_counts[0][i]);
    }

    /* Load mtr currents */
    i_mtr_ab[0] = mtrCountsToAmps(adc_counts[2][5]);
    i_mtr_ab[1] = mtrCountsToAmps(adc_counts[2][4]);
    i_mtr_ab[2] = mtrCountsToAmps(adc_counts[2][0]);
    i_mtr_ab[3] = mtrCountsToAmps(adc_counts[2][1]);

    i_mtr[0]    = mtrCountsToAmps(adc_counts[2][2]);
    i_mtr[1]    = mtrCountsToAmps(adc_counts[2][3]);

    /* Load thermocouples temps above */

    /* Load potentiometer degrees */
    for (int8_t i = 9; i >= 6; --i) {
        int8_t channel  = 9-i;
        if (channel < NUM_POTS) {  // Press board code uses 4 pots
        	epot[channel]   = potCountsToDegrees(adc_counts[2][i], channel);
        }
    }

    /* Load stepper motor position */
    for (uint8_t i = 0; i < NUM_TANKS; ++i) {
        mtr_pos[i] = curPos[i];
    }

    /* Load board currents and voltages */
    i_batt  = adc_counts[2][12]*ibatt_cal;
    e_batt  = adc_counts[2][13]*ebatt_cal;
    i3v     = adc_counts[2][10]*i3v_cal;
    e3v     = adc_counts[2][11]*e3v_cal;
    i5v     = adc_counts[0][8]*i5v_cal;
    e5v     = adc_counts[0][9]*e5v_cal;
}

float vlvCountsToVolts(uint16_t counts) {
    return counts*evlv_cal;
}

float vlvCountsToAmps(uint16_t counts) {
    return counts*ivlv_cal;
}

/*
float ptCountsToPsi(uint16_t counts, uint8_t pt_num) {
    return ( counts- pt_offset[pt_num]) *pt_slope[pt_num] - pt_ambients[pt_num];
	//return 0;
}
*/

float mtrCountsToAmps(uint16_t counts) {
    return counts*imtr_cal;
}

float potCountsToDegrees(uint16_t counts, uint8_t pot_num) {
    // negative sign to align motor pos direction with pot pos direction
    return -EPOT_CHAR_SLOPE*(
                        (pot_cal_info.slope * (counts - pot_cal_info.offset))
                         - pot_ambients[pot_num] - EPOT_CHAR_OFFSET
                           );
}
