#ifndef INC_HARDWARE_H_
#define INC_HARDWARE_H_

#include "main.h"
#include "MAX31855.h"
#include "MAX11131.h"
//#include "comms.h"
#include "W25N01GV.h"
#include "globals.h"
//#include "pack_telem_defines.h"
#include "calibrations.h"
#include "constants.h"

/* Calibration Constants */
// TODO: update cals, these are taken from the EC
#define ibatt_cal   0.01611721  // checked
//#define ebatt_cal   0.00324707  // checked
//#define ebatt_cal   0.0032470703125  // calculated, not measured
#define ebatt_slope   (0.0038)  // measured
#define ebatt_offset  (-0.5689) // measured
//#define ebatt_offset	1.28 	//measured offset TODO: delete later
#define ibus_cal    0.01418500
#define ept_cal     0.000806
#define evlv_cal    0.00324707  // checked
#define ivlv_cal    0.00322265  // checked
#define imtr_cal    0.00322265
#define epot_slope  0.88311388  //counts to degrees
#define epot_offset 4.096       //counts
#define e5v_cal     0.00161132  // checked
#define e3v_cal     0.00161132  // checked
#define i5v_cal     1.00000000  // checked
#define i3v_cal     1.00000000  // checked
#define tbrd_offset 600.000000
#define tbrd_slope  0.12400000

#define EPOT_DMAX			3610	// degs
#define EPOT_DMIN			0		// degss
#define EPOT_RES_LEAD		0.9		// ohms
#define EPOT_RES_POT		1000	// ohms
#define EPOT_RES_POT_MIN	1		// ohms
#define EPOT_MAX_COUNTS		4096	// counts
#define EPOT_CHAR_OFFSET	0.0f	// from testing characterization
#define EPOT_CHAR_SLOPE		1.0f // from testing characterization

// Motor Datasheet
#define degPerStep  (float) 0.35
#define stepPerDeg  (float) 2.85714


extern uint16_t adc_counts[3][16];  // array for receiving adc conversions
extern GPIO_MAX11131_Pinfo adc_pins[3];
extern MAX31855_Pinfo tc_pins[5];

// Pot ambients
extern float pot_ambients[2];

extern float maxPos;

/* Firmware/Hardware specific variables */

typedef struct Potentiometer_Cal {
	float slope;
	float offset;
} Potentiometer_Cal;

typedef struct ADC_Cal {
	uint16_t slope[16];
	uint16_t offset[16];
} ADC_Cal;

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

extern Potentiometer_Cal pot_cal_info;

extern Stepper_Pinfo stepper_pinfo[2]; // information about stepper motor pin outs


extern Stepper_Ramping stepper;
extern volatile int8_t curDir[NUM_TANKS];                  // direction of motor spinning
extern volatile float targetPos[NUM_TANKS];                // target position for motor
extern volatile float curPos[NUM_TANKS];                   // current position for motor
extern volatile uint8_t manual_stepper_pos_override[NUM_TANKS]; // override for stepping motor


//W25N01GV_Flash flash; // Flash interface variable

/* Wrapper files for interfacing with the firmware libraries */

void init_hardware();

uint8_t isMotorAtPos(float target, uint8_t tank_num);

void handleMotorStepping(uint8_t motor_num);

void turn_stepper_motor(TIM_HandleTypeDef *htimx, uint8_t motor_num, int8_t direction, uint8_t step_stage);

void moveMotorToPos(float deg, uint8_t motor_num);

void computeStepperRamp(Stepper_Ramping* stepper);

void changeStepperPeriod(TIM_HandleTypeDef *htimx, Stepper_Ramping* stepper);

void setValve(uint32_t vlv_num, uint8_t state);

void initAdcs(SPI_HandleTypeDef* hspix, GPIO_MAX11131_Pinfo *adc_pins);

void readAdcs(SPI_HandleTypeDef* hspix, GPIO_MAX11131_Pinfo* adc_pins, uint16_t (*adc_counts)[16]);

void updatePeripherals(uint16_t (*adc_counts)[16]);

/*
void send_telem_packet(UART_HandleTypeDef* huartx, CLB_Packet_Header* header, uint8_t* buffer, int32_t buffer_sz);

void save_telem_to_flash(W25N01GV_Flash* flash);

void send_telem_from_flash(W25N01GV_Flash* flash, UART_HandleTypeDef* huartx);
*/

void readThermocouples(SPI_HandleTypeDef* hspix, MAX31855_Pinfo *tc_pins,
                                                            uint8_t tc_num);
/*
void write_constants_to_flash(W25N01GV_Flash* flash);

uint8_t read_constants_from_flash(W25N01GV_Flash* flash);
*/

// Private Helper Functions

float vlvCountsToVolts(uint16_t counts);

float vlvCountsToAmps(uint16_t counts);

/*
float ptCountsToPsi(uint16_t counts, uint8_t pt_num);
*/

float mtrCountsToAmps(uint16_t counts);

float potCountsToDegrees(uint16_t counts, uint8_t pot_num);

#endif
