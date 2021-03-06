/// BEGIN AUTOGENERATED SECTION - MODIFICATIONS TO THIS CODE WILL BE OVERWRITTEN

/// globals.h
/// Autogenerated by firmware-libraries/SerialComms/python/telem_file_generator.py on Fri Aug  6 01:55:37 2021

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_
#include <stdint.h>

extern enum States STATE;
extern float e_batt;
extern float i_batt;
extern uint32_t valve_states;
extern float e3v;
extern float e5v;
extern float i5v;
extern float i3v;
extern uint32_t status_flags;
extern float lox_control_pressure;
extern float fuel_control_pressure;
extern float copv_control_pressure;
extern float init_motor_pos_deg_correction_factor;
extern int32_t state_rem_duration;
extern uint8_t telem_rate;
extern uint8_t adc_rate;
extern uint32_t flash_mem;
extern uint8_t LOGGING_ACTIVE;
extern float pressure[20];
extern float evlv[14];
extern float tc[13];
extern volatile float epot[2];

/// END AUTOGENERATED SECTION - USER CODE GOES BELOW THIS LINE

enum States {
    Manual              = 0,
    Armed               = 1,
    AutoPress           = 2,
    Startup             = 3,
    Ignition            = 4,
    Hotfire             = 5,
    Abort               = 6,
    Post                = 7,
    Safe                = 8,
	IgnitionFail        = 9,
    Continue            = 255
};

#endif /*INC_GLOBALS_H*/
